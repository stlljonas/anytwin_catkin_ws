/*!
 * @file    TorqueControllerJointPositionVelocity.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityPid.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


TorqueControllerJointPositionVelocityPid::TorqueControllerJointPositionVelocityPid(double positionGain, double velocityGain) :
    ControllerBase(),
    pGain_(positionGain),
    dGain_(velocityGain)
{

}

TorqueControllerJointPositionVelocityPid::~TorqueControllerJointPositionVelocityPid()
{

}

bool TorqueControllerJointPositionVelocityPid::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt)
{
  return true;
}

bool TorqueControllerJointPositionVelocityPid::advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt)
{
  commandOut.setJointPosition(saturate(commandIn.getJointPosition(), commandIn.getJointPositionMin(), commandIn.getJointPositionMax()));
  commandOut.setJointVelocity(saturate(commandIn.getJointVelocity(), commandIn.getJointVelocityMin(), commandIn.getJointVelocityMax()));

  double jointTorque = (commandOut.getJointPosition()-state.getJointPosition())*pGain_
                     + (commandOut.getJointVelocity()-state.getJointVelocity())*dGain_;

  jointTorque = saturate(jointTorque, commandIn.getJointTorqueMin(), commandIn.getJointTorqueMax());
  commandOut.setJointTorque(jointTorque);
  return true;
}

double TorqueControllerJointPositionVelocityPid::getJointPositionPGain() const
{
  return pGain_;
}

void TorqueControllerJointPositionVelocityPid::setJointPositionPGain(double positionGain)
{
  pGain_ = positionGain;
}

double TorqueControllerJointPositionVelocityPid::getJointPositionDGain() const
{
  return dGain_;
}

void TorqueControllerJointPositionVelocityPid::setJointPositionDGain(double velocityGain)
{
  dGain_ = velocityGain;
}


} // controller
} // series_elastic_actuator_sim
