/*!
 * @file     RocoCommand.cpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Dec, 2014
 */

#include "anymal_roco/RocoCommand.hpp"
#include <cmath>
#include <iomanip>
#include <message_logger/message_logger.hpp>
#include <romo_std/common/container_utils.hpp>


#include "signal_logger/signal_logger.hpp"

namespace anymal_roco {

using AD = anymal_description::AnymalDescription;

RocoCommand::RocoCommand():
  roco::CommandInterface()
{
  romo_std::fillContainer<AD>(actuatorCommands_);
  romo_std::fillBranchNodePtrContainer<AD>(branchNodeActuatorCommands_, actuatorCommands_);
  romo_std::fillNodeBranchPtrContainer<AD>(nodeBranchActuatorCommands_, actuatorCommands_);
}

bool RocoCommand::limitCommand() {
  bool isCommandOk = true;

  for (const auto& actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    auto& actuator = actuatorCommands_[actuatorEnum];

    switch (actuator.getMode()) {

      case Mode::MODE_FREEZE:
      case Mode::MODE_DISABLE:
      case Mode::MODE_MOTOR_VELOCITY:
      case Mode::MODE_JOINT_POSITION:
      case Mode::MODE_JOINT_TORQUE:
      case Mode::MODE_JOINT_POSITION_VELOCITY:
      case Mode::MODE_JOINT_POSITION_VELOCITY_TORQUE:
      case Mode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS:
        break;
      case Mode::MODE_CURRENT:
      case Mode::MODE_MOTOR_POSITION:
      case Mode::MODE_GEAR_VELOCITY:
      case Mode::MODE_GEAR_POSITION:
      case Mode::MODE_JOINT_VELOCITY:
        MELO_WARN_STREAM("Desired control mode \"" << actuator.getMode() << "\" for actuator " << AD::mapKeyEnumToKeyName(actuatorEnum) << " is currently not supported!");
        isCommandOk = false;
        break;
      default:
        MELO_WARN_STREAM("Desired control mode \"" << actuator.getMode() << "\" for actuator " << AD::mapKeyEnumToKeyName(actuatorEnum) << " is invalid!");
        isCommandOk = false;
    }

    if (!actuator.isFinite()) {
      MELO_WARN_STREAM("Desired actuator command has infinite values: " << std::endl << actuator);
      actuator.setMode(Mode::MODE_FREEZE);
      isCommandOk = false;
    }

    // Warn user if command is not within limits.
    if (actuator.getMode() != Mode::MODE_DISABLE && actuator.getMode() != Mode::MODE_FREEZE) {
      if (!actuator.isWithinLimits()) {
        MELO_WARN_THROTTLE_STREAM(4,"Desired actuator command is not within limits: " << std::endl << actuator);
      }
    }

    // limit values
    actuator.limit();

  }
  return isCommandOk;
}


const anymal_model::ActuatorCommandRobotContainer& RocoCommand::getActuatorCommands() const {
  return actuatorCommands_;
}

anymal_model::ActuatorCommandRobotContainer& RocoCommand::getActuatorCommands() {
  return actuatorCommands_;
}

void RocoCommand::setActuatorCommands(const anymal_model::ActuatorCommandRobotContainer& commands) {
  actuatorCommands_ = commands;
}

const anymal_model::ActuatorCommandPtrBranchNodeContainer& RocoCommand::getBranchNodeActuatorCommands() const {
  return branchNodeActuatorCommands_;
}

anymal_model::ActuatorCommandPtrBranchNodeContainer& RocoCommand::getBranchNodeActuatorCommands() {
  return branchNodeActuatorCommands_;
}

const anymal_model::ActuatorCommandPtrNodeBranchContainer& RocoCommand::getNodeBranchActuatorCommands() const {
  return nodeBranchActuatorCommands_;
}
anymal_model::ActuatorCommandPtrNodeBranchContainer& RocoCommand::getNodeBranchActuatorCommands() {
  return nodeBranchActuatorCommands_;
}

anymal_model::ContactForceCalibratorCommandContainer& RocoCommand::getForceCalibratorCommands() {
  return forceCalibratorCommands_;
}
const anymal_model::ContactForceCalibratorCommandContainer& RocoCommand::getForceCalibratorCommands() const {
 return forceCalibratorCommands_;
}

void RocoCommand::addVariablesToLog(bool updateLogger) {
  std::string ns{"/command/"};
//  for (auto& command: actuatorCommands_) {
  for (const auto& actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorName = actuatorKey.getName();
    const auto& command = actuatorCommands_[actuatorEnum];
    signal_logger::add(command.getJointPosition(), std::string{"desJointPos_"} + actuatorName, ns);
    signal_logger::add(command.getJointVelocity(), std::string{"desJointVel_"} + actuatorName, ns);
    signal_logger::add(command.getJointTorque(), std::string{"desJointTor_"} + actuatorName, ns);
    signal_logger::add(command.getPidGainsP(), std::string{"pidGainsP_"} + actuatorName, ns);
    signal_logger::add(command.getPidGainsI(), std::string{"pidGainsI_"} + actuatorName, ns);
    signal_logger::add(command.getPidGainsD(), std::string{"pidGainsD_"} + actuatorName, ns);
    signal_logger::add(command.getModeEnum(), std::string{"mode_"} + actuatorName, ns);
  }
  ns = std::string{"/command/force_calib/"};
  std::string names[] {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  int i=0;
  for (auto& command: forceCalibratorCommands_) {
    signal_logger::add(command.cmdCalibrate_, std::string{"cmdCalibrate_"} + names[i], ns);
    signal_logger::add(command.cmdContinue_, std::string{"cmdContinue_"} + names[i], ns);
    signal_logger::add(command.cmdStart_, std::string{"cmdStart_"} + names[i], ns);
    signal_logger::add(command.enableOutlierDetector_, std::string{"enableOutlierDetector_"} + names[i], ns);
    signal_logger::add(command.numSamples_, std::string{"numSamples_"} + names[i], ns);
    signal_logger::add(command.numGoodSamples_, std::string{"numGoodSamples_"} + names[i], ns);
    ++i;
  }
}

std::ostream& operator <<(std::ostream& out, const RocoCommand& command) {
//  for (auto& actuator : command.getActuatorCommands()) {
//    out << std::to_string(actuator.getId()) << ": " << actuator.getName() << ": mode: " << actuator.getModeName()   << " th: " << std::fixed << std::setw(8) << std::setprecision(4) << std::showpoint << std::right << actuator.getJointPosition()  << " thd: " << std::fixed << std::setw(8) << std::setprecision(4) << std::showpoint << std::right << actuator.getJointVelocity()  << " torque: " << std::fixed << std::setw(8) << std::setprecision(4) << std::showpoint << std::right << actuator.getTorque()  << std::endl;
//  }
  return out;
}

}
