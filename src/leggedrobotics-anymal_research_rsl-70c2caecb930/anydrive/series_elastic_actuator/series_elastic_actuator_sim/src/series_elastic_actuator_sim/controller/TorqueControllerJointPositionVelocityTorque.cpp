/*
 * TorqueControllerJointPositionVelocityTorque.cpp
 *
 *  Created on: Oct 27, 2016
 *      Author: Remo Diethelm
 */

// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityTorque.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


TorqueControllerJointPositionVelocityTorque::TorqueControllerJointPositionVelocityTorque() {}

TorqueControllerJointPositionVelocityTorque::~TorqueControllerJointPositionVelocityTorque() {}

bool TorqueControllerJointPositionVelocityTorque::initialize(
    const SeActuatorCommand& command,
    const SeActuatorState& state,
    double dt)
{
  iError_ = 0.0;
  return true;
}

bool TorqueControllerJointPositionVelocityTorque::advance(
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

  double jointTorque = pGain_*pError + iGain_*iError_ + dGain_*dError + commandIn.getJointTorque();

  jointTorque = saturate(jointTorque, commandIn.getJointTorqueMin(), commandIn.getJointTorqueMax());
  commandOut.setJointTorque(jointTorque);

  return true;
}

double TorqueControllerJointPositionVelocityTorque::getJointPositionPGain() const
{
  return pGain_;
}

double TorqueControllerJointPositionVelocityTorque::getJointPositionDGain() const
{
  return dGain_;
}

double TorqueControllerJointPositionVelocityTorque::getJointPositionIGain() const
{
  return iGain_;
}

void TorqueControllerJointPositionVelocityTorque::setJointPositionPGain(double pGain)
{
  pGain_ = pGain;
}

void TorqueControllerJointPositionVelocityTorque::setJointPositionDGain(double dGain)
{
  dGain_ = dGain;
}

void TorqueControllerJointPositionVelocityTorque::setJointPositionIGain(double iGain)
{
  iGain_ = iGain;
}


} // controller
} // series_elastic_actuator_sim
