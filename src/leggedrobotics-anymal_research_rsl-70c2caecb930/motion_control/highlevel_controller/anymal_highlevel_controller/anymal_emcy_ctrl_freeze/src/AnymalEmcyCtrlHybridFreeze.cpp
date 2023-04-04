/*!
 * @file        AnymalEmcyCtrlHybridFreeze.hpp
 * @author      Valentin Yuryev
 * @date        Dec 06, 2019
 * @brief       Implementation of controller that disables if moving and freezes if in place
 */

#include "anymal_emcy_ctrl_freeze/AnymalEmcyCtrlHybridFreeze.hpp"

#include <rocoma_plugin/rocoma_plugin.hpp>

#include <tinyxml_tools/tinyxml_tools.hpp>

ROCOMA_EXPORT_EMERGENCY_CONTROLLER(HybridFreeze, anymal_roco::RocoState, anymal_roco::RocoCommand, robot_controller::AnymalEmcyCtrlHybridFreeze)


namespace robot_controller {

AnymalEmcyCtrlHybridFreeze::AnymalEmcyCtrlHybridFreeze() : Base(), emergencyState_(EmergencyState::STATIONARY),
                                                                   velocityThreshold_(0.5),
                                                                   previousCommand_(Eigen::Matrix<double, 12, 1>::Zero()),
                                                                   alpha_(0.005),
                                                                   timeout_(1.5),
                                                                   dampingDuration_(0.0) {
  this->setName("hybrid_freeze");
}

bool AnymalEmcyCtrlHybridFreeze::create(double dt) {
  this->setIsCheckingState(false);
  return loadParameters(dt);
}

bool AnymalEmcyCtrlHybridFreeze::loadParameters(double dt) {
  parameterSet_.reset(new loco::ParameterSet());

  std::string parameterFile = this->getParameterPath() + "/HybridFreeze.xml";

  if (!parameterSet_->loadXmlDocument(parameterFile)) {
    MELO_ERROR_STREAM("Could not load parameter file: " << parameterFile);
    return false;
  }

  auto locoControllerHandle = parameterSet_->getHandle().FirstChild("LocomotionController");

  if (!tinyxml_tools::loadParameter(pidGains_.pGain_, locoControllerHandle.FirstChild("Gains"), "pGain")) {
    MELO_FATAL("Could not load gains parameters!")
    return false;
  }

  if (!tinyxml_tools::loadParameter(pidGains_.iGain_, locoControllerHandle.FirstChild("Gains"), "iGain")) {
    MELO_FATAL("Could not load gains parameters!")
    return false;
  }

  if (!tinyxml_tools::loadParameter(pidGains_.dGain_, locoControllerHandle.FirstChild("Gains"), "dGain")) {
    MELO_FATAL("Could not load gains parameters!")
    return false;
  }

  if (!tinyxml_tools::loadParameter(velocityThreshold_, locoControllerHandle.FirstChild("velocity_threshold"), "velocity")) {
    MELO_FATAL("Could not load velocity_threshold parameters!")
    return false;
  }

  double timeConstant = 0.5;
  if (!tinyxml_tools::loadParameter(timeConstant, locoControllerHandle.FirstChild("filter"), "time_constant")) {
    MELO_FATAL("Could not load filter parameters!")
    return false;
  }
  alpha_ = std::min(1., std::max(0., dt / timeConstant));

  if (!tinyxml_tools::loadParameter(timeout_, locoControllerHandle.FirstChild("timeout"), "seconds")) {
    MELO_FATAL("Could not load timeout parameters!")
    return false;
  }

  return true;
}

bool AnymalEmcyCtrlHybridFreeze::initializeFast(double dt) {
  dampingDuration_ = 0.0;
  previousCommand_ = getState().getAnymalModel().getState().getJointPositions().vector();
  return advanceState() && advance(dt);
}

bool AnymalEmcyCtrlHybridFreeze::advance(double dt) {
  switch (emergencyState_) {
    case EmergencyState::MOVING:
      dampingDuration_ += dt;
      if (dampingDuration_ >= timeout_) {
        emergencyState_ = EmergencyState::STATIONARY; // Switch to freeze controller after some seconds. Dampening controller drifts.
      }
      return dampDrives();
    case EmergencyState::STATIONARY:
      return freezeDrives();
    default:
      MELO_FATAL("[AnymalEmcyCtrlHybridFreeze] Unknown EmergencyState, controller failed.")
      return false;
  }
}

bool AnymalEmcyCtrlHybridFreeze::advanceState() {
  if (isMoving()) {
    emergencyState_ = EmergencyState::MOVING;
  } else {
    emergencyState_ = EmergencyState::STATIONARY;
  }
  return true;
}

bool AnymalEmcyCtrlHybridFreeze::isMoving() {
  boost::unique_lock<boost::shared_mutex> lock(getStateMutex());
  return getState().getAnymalModelPtr()->getState().getJointVelocities().norm() > velocityThreshold_;
}

bool AnymalEmcyCtrlHybridFreeze::dampDrives() {
  auto& command = getCommand();
  const auto& jointPositions = getState().getAnymalModel().getState().getJointPositions().vector();
  Eigen::Matrix<double, 12, 1> newCommand = alpha_ * jointPositions + (1 - alpha_) * previousCommand_;
  for (const auto actuatorKey : anymal_model::AD::getActuatorKeys()) {
    const auto actuatorId = actuatorKey.getId();
    const auto actuatorEnum = actuatorKey.getEnum();
    command.getActuatorCommands()[actuatorEnum].setMode(Command::Mode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS);
    command.getActuatorCommands()[actuatorEnum].setJointPosition(newCommand[actuatorId]);
    command.getActuatorCommands()[actuatorEnum].setPidGains(pidGains_);
    command.getActuatorCommands()[actuatorEnum].setJointVelocity(0.0);
    command.getActuatorCommands()[actuatorEnum].setJointTorque(0.0);
  }
  previousCommand_ = newCommand;
  return true;
}

bool AnymalEmcyCtrlHybridFreeze::freezeDrives() {
  for (auto& command : getCommand().getActuatorCommands()) {
    command.setMode(command.Mode::MODE_FREEZE);
    command.setMotorVelocity(0.0);
    command.setJointVelocity(0.0);
    command.setJointTorque(0.0);
  }
  return true;
}

} /* namespace robot_controller */
