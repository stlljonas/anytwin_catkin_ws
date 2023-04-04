/*
 * TorqueControllerImpedance.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: Dario Bellicoso
 */

// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/TorqueControllerImpedance.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


TorqueControllerImpedance::TorqueControllerImpedance(double springGain,
                                                     double damperGain,
                                                     double inertiaGain)
    : springGain_(springGain),
      damperGain_(damperGain),
      inertiaGain_(inertiaGain)
{

}

TorqueControllerImpedance::~TorqueControllerImpedance()
{
}

bool TorqueControllerImpedance::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt) {
  return true;
}

bool TorqueControllerImpedance::advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt) {

  commandOut.setJointPosition(saturate(commandIn.getJointPosition(), commandIn.getJointPositionMin(), commandIn.getJointPositionMax()));
  commandOut.setJointVelocity(saturate(commandIn.getJointVelocity(), commandIn.getJointVelocityMin(), commandIn.getJointVelocityMax()));

  double jointTorque = commandIn.getJointTorque()
      + springGain_*(commandOut.getJointPosition() - state.getJointPosition())
      + damperGain_*(commandOut.getJointVelocity() - state.getJointVelocity());

  jointTorque = saturate(jointTorque, commandIn.getJointTorqueMin(), commandIn.getJointTorqueMax());
  commandOut.setJointTorque(jointTorque);

  return true;
}

double TorqueControllerImpedance::getSpringGain() const {
  return springGain_;
}

double TorqueControllerImpedance::getDamperGain() const {
  return damperGain_;
}

double TorqueControllerImpedance::getInertiaGain() const {
  return inertiaGain_;
}

void TorqueControllerImpedance::setSpringGain(double springGain) {
  springGain_ = springGain;
}

void TorqueControllerImpedance::setDamperGain(double damperGain) {
  damperGain_ = damperGain;
}

void TorqueControllerImpedance::setInertiaGain(double inertiaGain) {
  inertiaGain_ = inertiaGain;
}


} // controller
} // series_elastic_actuator_sim
