/*!
 * @file    MotorVelocityControllerTorque.cpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/MotorVelocityControllerTorque.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


MotorVelocityControllerTorque::MotorVelocityControllerTorque(double torqueGain) :
    ControllerBase(),
    torqueGain_(torqueGain)
{

}

MotorVelocityControllerTorque::~MotorVelocityControllerTorque()
{

}

bool MotorVelocityControllerTorque::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt)
{
  return true;
}

bool MotorVelocityControllerTorque::advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt)
{
  commandOut.setJointTorque(saturate(commandIn.getJointTorque(), commandIn.getJointTorqueMin(), commandIn.getJointTorqueMax()));

  double motorVelocity = (commandOut.getJointTorque()-state.getJointTorque())*torqueGain_;
  motorVelocity = saturate(motorVelocity, commandIn.getMotorVelocityMin(), commandIn.getMotorVelocityMax());
  commandOut.setMotorVelocity(motorVelocity);
  return true;
}

double MotorVelocityControllerTorque::getTorqueGain() const
{
  return torqueGain_;
}

void MotorVelocityControllerTorque::setTorqueGain(double torqueGain)
{
  torqueGain_ = torqueGain;
}


} // controller
} // series_elastic_actuator_sim
