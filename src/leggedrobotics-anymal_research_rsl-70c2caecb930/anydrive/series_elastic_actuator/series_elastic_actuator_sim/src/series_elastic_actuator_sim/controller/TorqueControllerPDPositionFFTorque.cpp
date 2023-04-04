/*
 * TorqueControllerPDPositionFFTorque.cpp
 *
 *  Created on: Oct 16, 2015
 *      Author: Georg Wiedebach
 */

// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/TorqueControllerPDPositionFFTorque.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


TorqueControllerPDPositionFFTorque::TorqueControllerPDPositionFFTorque()
    : controllerJointPosition_(), ControllerBase()
{

}

TorqueControllerPDPositionFFTorque::~TorqueControllerPDPositionFFTorque()
{

}

bool TorqueControllerPDPositionFFTorque::initialize(
    const SeActuatorCommand& command,
    const SeActuatorState& state, double dt)
{
	return controllerJointPosition_.initialize(command, state, dt);
}

bool TorqueControllerPDPositionFFTorque::advance(
    SeActuatorCommand& commandOut,
    const SeActuatorCommand& commandIn,
    const SeActuatorState& state, double dt)
{
	bool ret = controllerJointPosition_.advance(commandOut, commandIn, state, dt);
	double jointTorque = commandOut.getJointTorque() + commandIn.getJointTorque();

	jointTorque = saturate(jointTorque, commandIn.getJointTorqueMin(), commandIn.getJointTorqueMax());
	commandOut.setJointTorque(jointTorque);

	return ret;
}

void TorqueControllerPDPositionFFTorque::setPositionGain(double positionGain)
{
	controllerJointPosition_.setJointPositionPGain(positionGain);
}

void TorqueControllerPDPositionFFTorque::setVelocityGain(double velocityGain)
{
	controllerJointPosition_.setJointPositionDGain(velocityGain);
}


} // controller
} // series_elastic_actuator_sim
