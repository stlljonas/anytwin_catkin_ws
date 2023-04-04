/*!
 * @file    SeActuatorPerfectTorqueSource.cpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator sim
#include "series_elastic_actuator_sim/SeActuatorPerfectTorqueSource.hpp"


namespace series_elastic_actuator_sim {


SeActuatorPerfectTorqueSource::SeActuatorPerfectTorqueSource() {}

SeActuatorPerfectTorqueSource::~SeActuatorPerfectTorqueSource() {}

bool SeActuatorPerfectTorqueSource::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt)
{
  controllers_.emplace(SeActuatorCommand::SeActuatorMode::MODE_DISABLE, &controllerDisable_);
  controllers_.emplace(SeActuatorCommand::SeActuatorMode::MODE_FREEZE, &controllerFreeze_);
  controllers_.emplace(SeActuatorCommand::SeActuatorMode::MODE_JOINT_TORQUE, &controllerTorque_);
  controllers_.emplace(SeActuatorCommand::SeActuatorMode::MODE_JOINT_POSITION, &controllerJointPosition_);
  controllers_.emplace(SeActuatorCommand::SeActuatorMode::MODE_JOINT_POSITION_VELOCITY, &controllerJointPositionVelocity_);
  controllers_.emplace(SeActuatorCommand::SeActuatorMode::MODE_JOINT_POSITION_VELOCITY_TORQUE, &controllerJointPositionVelocityTorque_);
  controllers_.emplace(SeActuatorCommand::SeActuatorMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS, &controllerJointPositionVelocityTorquePidGains_);
  return initializeControllers(command, state, dt);
}

bool SeActuatorPerfectTorqueSource::advance(SeActuatorReading& reading, const SeActuatorCommand& command, double dt)
{
  if (!advanceController(reading, command, dt)) {
    return false;
  }

  reading.getState().setJointTorque(reading.getCommanded().getJointTorque());
  return true;
}

controller::TorqueControllerDisable& SeActuatorPerfectTorqueSource::getControllerDisable() {
  return controllerDisable_;
}

controller::TorqueControllerFreeze& SeActuatorPerfectTorqueSource::getControllerFreeze() {
  return controllerFreeze_;
}

controller::TorqueControllerTorque& SeActuatorPerfectTorqueSource::getControllerTorque() {
  return controllerTorque_;
}

controller::TorqueControllerJointPositionVelocityPid& SeActuatorPerfectTorqueSource::getControllerJointPosition() {
  return controllerJointPosition_;
}

controller::TorqueControllerJointPositionVelocityPid& SeActuatorPerfectTorqueSource::getControllerJointPositionVelocity() {
  return controllerJointPositionVelocity_;
}

controller::TorqueControllerJointPositionVelocityTorque& SeActuatorPerfectTorqueSource::getControllerJointPositionVelocityTorque() {
  return controllerJointPositionVelocityTorque_;
}

controller::TorqueControllerJointPositionVelocityTorquePidGains& SeActuatorPerfectTorqueSource::getControllerJointPositionVelocityTorquePidGains() {
  return controllerJointPositionVelocityTorquePidGains_;
}


} // series_elastic_actuator_sim

