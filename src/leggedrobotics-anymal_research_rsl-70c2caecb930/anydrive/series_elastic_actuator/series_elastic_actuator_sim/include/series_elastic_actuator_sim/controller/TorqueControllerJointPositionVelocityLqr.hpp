/*
 * TorqueControllerJointPositionVelocityLqr.hpp
 *
 *  Created on: Jul 1, 2015
 *      Author: Dario Bellicoso
 */

#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/ControllerBase.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


class TorqueControllerJointPositionVelocityLqr : public ControllerBase
{
 public:
  TorqueControllerJointPositionVelocityLqr(
      double jointPositionGain = 0.0,
      double jointVelocityGain = 0.0,
      double motorPositionGain = 0.0);
  virtual ~TorqueControllerJointPositionVelocityLqr();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt);

  double getMotorPositionGain() const;
  double getJointPositionGain() const;
  double getJointVelocityGain() const;

  void setMotorPositionGain(double motorPositionGain);
  void setJointPositionGain(double jointPositionGain);
  void setJointVelocityGain(double jointVelocityGain);

 protected:
  double jointPositionGain_;
  double jointVelocityGain_;
  double motorPositionGain_;
};


} // controller
} // series_elastic_actuator_sim

