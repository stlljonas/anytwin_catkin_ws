/*
 * TorqueControllerJointPositionVelocityLqr.cpp
 *
 *  Created on: Jul 1, 2015
 *      Author: Dario Bellicoso
 */

// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityLqr.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


TorqueControllerJointPositionVelocityLqr::TorqueControllerJointPositionVelocityLqr(
    double jointPositionGain,
    double jointVelocityGain,
    double motorPositionGain) :
    ControllerBase(),
    jointPositionGain_(jointPositionGain),
    jointVelocityGain_(jointVelocityGain),
    motorPositionGain_(motorPositionGain)
{

}

TorqueControllerJointPositionVelocityLqr::~TorqueControllerJointPositionVelocityLqr()
{

}

bool TorqueControllerJointPositionVelocityLqr::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt) {
  return true;
}

bool TorqueControllerJointPositionVelocityLqr::advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt) {
  commandOut.setJointPosition(saturate(commandIn.getJointPosition(), commandIn.getJointPositionMin(), commandIn.getJointPositionMax()));
  commandOut.setJointVelocity(saturate(commandIn.getJointVelocity(), commandIn.getJointVelocityMin(), commandIn.getJointVelocityMax()));
//  commandOut.setMotorPosition(saturate(commandIn.getPosition(), commandIn.getMotorPositionMin(), commandIn.getMotorPositionMax()));

  double jointTorque = (commandOut.getJointPosition()-state.getJointPosition())*jointPositionGain_
//                     + (commandOut.getMotorPosition()-state.getMotorPosition())*motorPositionGain_
                     + (commandOut.getJointVelocity()-state.getJointVelocity())*jointVelocityGain_
                     + commandOut.getJointVelocity();

  jointTorque = saturate(jointTorque, commandIn.getJointTorqueMin(), commandIn.getJointTorqueMax());
  commandOut.setJointTorque(jointTorque);
  return true;
}

double TorqueControllerJointPositionVelocityLqr::getMotorPositionGain() const {
  return motorPositionGain_;
}

void TorqueControllerJointPositionVelocityLqr::setMotorPositionGain(double motorPositionGain) {
  motorPositionGain_ = motorPositionGain;
}

double TorqueControllerJointPositionVelocityLqr::getJointPositionGain() const {
  return jointPositionGain_;
}

void TorqueControllerJointPositionVelocityLqr::setJointPositionGain(double jointPositionGain) {
  jointPositionGain_ = jointPositionGain;
}

double TorqueControllerJointPositionVelocityLqr::getJointVelocityGain() const {
  return jointVelocityGain_;
}

void TorqueControllerJointPositionVelocityLqr::setJointVelocityGain(double jointVelocityGain) {
  jointVelocityGain_ = jointVelocityGain;
}


} // controller
} // series_elastic_actuator_sim
