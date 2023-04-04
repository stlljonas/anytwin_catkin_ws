/*!
 * @file    TorqueControllerFreeze.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/TorqueControllerFreeze.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


TorqueControllerFreeze::TorqueControllerFreeze(double positionGain, double velocityGain) :
    ControllerBase(),
    positionGain_(positionGain),
    velocityGain_(velocityGain)
{

}

TorqueControllerFreeze::~TorqueControllerFreeze()
{

}

bool TorqueControllerFreeze::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt)
{
  // We take the desired joint position with an offset to the measured joint position in order to avoid "collapsing" slightly.
  jointPositionDes_ = state.getJointPosition() + state.getJointTorque()/positionGain_;
  return true;
}

bool TorqueControllerFreeze::advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt)
{
  const double jointTorque =
      (jointPositionDes_ - state.getJointPosition())*positionGain_ +
                         - state.getJointVelocity() *velocityGain_;
  commandOut.setJointTorque(jointTorque);
  return true;
}

double TorqueControllerFreeze::getJointPositionPGain() const
{
  return positionGain_;
}

void TorqueControllerFreeze::setJointPositionPGain(double positionGain)
{
  positionGain_ = positionGain;
}

double TorqueControllerFreeze::getJointPositionDGain() const
{
  return velocityGain_;
}

void TorqueControllerFreeze::setJointPositionDGain(double velocityGain)
{
  velocityGain_ = velocityGain;
}


} // controller
} // series_elastic_actuator_sim
