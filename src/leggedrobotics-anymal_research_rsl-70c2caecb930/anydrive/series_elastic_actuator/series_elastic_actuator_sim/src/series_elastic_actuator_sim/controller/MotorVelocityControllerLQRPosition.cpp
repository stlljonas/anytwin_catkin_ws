/*!
 * @file    MotorVelocityControllerLQRPosition.cpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/MotorVelocityControllerLQRPosition.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


MotorVelocityControllerLQRPosition::MotorVelocityControllerLQRPosition(double jointPositionGain, double jointVelocityGain, double motorPositionGain) :
    ControllerBase(),
    jointPositionGain_(jointPositionGain),
    jointVelocityGain_(jointVelocityGain),
    motorPositionGain_(motorPositionGain)
{

}

MotorVelocityControllerLQRPosition::~MotorVelocityControllerLQRPosition()
{

}

bool MotorVelocityControllerLQRPosition::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt)
{
  return true;
}

bool MotorVelocityControllerLQRPosition::advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt)
{
  commandOut.setJointPosition(saturate(commandIn.getJointPosition(), commandIn.getJointPositionMin(), commandIn.getJointPositionMax()));
  commandOut.setJointVelocity(saturate(commandIn.getJointVelocity(), commandIn.getJointVelocityMin(), commandIn.getJointVelocityMax()));
//  commandOut.setMotorPosition(saturate(commandIn.getPosition(), commandIn.getMotorPositionMin(), commandIn.getMotorPositionMax()));
//
//  double motorVelocity = (commandOut.getJointPosition()-state.getJointPosition())*jointPositionGain_
//                                  + (commandOut.getPosition()-state.getMotorPosition())*motorPositionGain_
//                                  + (commandOut.getJointVelocity()-state.getJointVelocity())*jointVelocityGain_
//                                  + commandOut.getJointVelocity();
//
//  motorVelocity = saturate(motorVelocity, commandIn.getMotorVelocityMin(), commandIn.getMotorVelocityMax());
//  commandOut.setVelocity(motorVelocity);
  return true;
}

double MotorVelocityControllerLQRPosition::getMotorPositionGain() const
{
  return motorPositionGain_;
}

void MotorVelocityControllerLQRPosition::setMotorPositionGain(double motorPositionGain)
{
  motorPositionGain_ = motorPositionGain;
}

double MotorVelocityControllerLQRPosition::getJointPositionGain() const
{
  return jointPositionGain_;
}

void MotorVelocityControllerLQRPosition::setJointPositionGain(double jointPositionGain)
{
  jointPositionGain_ = jointPositionGain;
}

double MotorVelocityControllerLQRPosition::getJointVelocityGain() const
{
  return jointVelocityGain_;
}

void MotorVelocityControllerLQRPosition::setJointVelocityGain(double jointVelocityGain)
{
  jointVelocityGain_ = jointVelocityGain;
}


} // controller
} // series_elastic_actuator_sim
