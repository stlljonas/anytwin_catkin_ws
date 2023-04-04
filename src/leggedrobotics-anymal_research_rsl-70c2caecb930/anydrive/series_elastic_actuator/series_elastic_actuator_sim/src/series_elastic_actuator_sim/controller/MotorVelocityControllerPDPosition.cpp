/*!
 * @file    MotorVelocityControllerPDPosition.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/MotorVelocityControllerPDPosition.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


MotorVelocityControllerPDPosition::MotorVelocityControllerPDPosition(double positionGain, double velocityGain) :
    ControllerBase(),
    positionGain_(positionGain),
    velocityGain_(velocityGain)
{

}

MotorVelocityControllerPDPosition::~MotorVelocityControllerPDPosition()
{

}

bool MotorVelocityControllerPDPosition::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt)
{
  return true;
}

bool MotorVelocityControllerPDPosition::advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt)
{
  commandOut.setJointPosition(saturate(commandIn.getJointPosition(), commandIn.getJointPositionMin(), commandIn.getJointPositionMax()));
  commandOut.setJointVelocity(saturate(commandIn.getJointVelocity(), commandIn.getJointVelocityMin(), commandIn.getJointVelocityMax()));

  double motorVelocity = (commandOut.getJointPosition()-state.getJointPosition())*positionGain_ +
                         (commandOut.getJointVelocity()-state.getJointVelocity())*velocityGain_ +
                         commandOut.getMotorVelocity();

//  motorVelocity = saturate(motorVelocity, commandIn.getMotorVelocityMin(), commandIn.getMotorVelocityMax());
  commandOut.setJointVelocity(motorVelocity);
  return true;
}

double MotorVelocityControllerPDPosition::getPositionGain() const
{
  return positionGain_;
}

void MotorVelocityControllerPDPosition::setPositionGain(double positionGain)
{
  positionGain_ = positionGain;
}

double MotorVelocityControllerPDPosition::getVelocityGain() const
{
  return velocityGain_;
}

void MotorVelocityControllerPDPosition::setVelocityGain(double velocityGain)
{
  velocityGain_ = velocityGain;
}


} // controller
} // series_elastic_actuator_sim
