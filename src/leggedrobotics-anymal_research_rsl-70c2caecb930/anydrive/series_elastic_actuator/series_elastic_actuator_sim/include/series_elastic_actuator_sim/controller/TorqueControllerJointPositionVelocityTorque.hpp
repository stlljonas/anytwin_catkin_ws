/*
 * TorqueControllerJointPositionVelocityTorque.hpp
 *
 *  Created on: Oct 27, 2016
 *      Author: Remo Diethelm
 */

#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/ControllerBase.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


class TorqueControllerJointPositionVelocityTorque : public ControllerBase
{
protected:
  double pGain_ = 0.0;
  double iGain_ = 0.0;
  double dGain_ = 0.0;

  double iError_ = 0.0;

public:
  TorqueControllerJointPositionVelocityTorque();
  virtual ~TorqueControllerJointPositionVelocityTorque();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt);

  double getJointPositionPGain() const;
  double getJointPositionIGain() const;
  double getJointPositionDGain() const;

  void setJointPositionPGain(double pGain);
  void setJointPositionIGain(double iGain);
  void setJointPositionDGain(double dGain);
};


} // controller
} // series_elastic_actuator_sim

