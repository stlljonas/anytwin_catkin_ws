/*
 * TorqueControllerJointPositionVelocityTorquePidGains.cpp
 *
 *  Created on: Oct 27, 2016
 *      Author: Remo Diethelm
 */

// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityTorquePidGains.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


TorqueControllerJointPositionVelocityTorquePidGains::TorqueControllerJointPositionVelocityTorquePidGains() {}

TorqueControllerJointPositionVelocityTorquePidGains::~TorqueControllerJointPositionVelocityTorquePidGains() {}

bool TorqueControllerJointPositionVelocityTorquePidGains::initialize(
    const SeActuatorCommand& command,
    const SeActuatorState& state,
    double dt)
{
  iError_ = 0.0;
  return true;
}

bool TorqueControllerJointPositionVelocityTorquePidGains::advance(
    SeActuatorCommand& commandOut,
    const SeActuatorCommand& commandIn,
    const SeActuatorState& state,
    double dt)
{
  commandOut.setJointPosition(saturate(commandIn.getJointPosition(), commandIn.getJointPositionMin(), commandIn.getJointPositionMax()));
  commandOut.setJointVelocity(saturate(commandIn.getJointVelocity(), commandIn.getJointVelocityMin(), commandIn.getJointVelocityMax()));

  const double pError = commandOut.getJointPosition() - state.getJointPosition();
  iError_ += pError*dt;
  const double dError = commandOut.getJointVelocity() - state.getJointVelocity();

  double jointTorque = commandIn.getPidGainsP()*pError + commandIn.getPidGainsI()*iError_ + commandIn.getPidGainsD()*dError + commandIn.getJointTorque();

  jointTorque = saturate(jointTorque, commandIn.getJointTorqueMin(), commandIn.getJointTorqueMax());
  commandOut.setJointTorque(jointTorque);

  return true;
}


} // controller
} // series_elastic_actuator_sim
