/*
 * TorqueControllerJointPositionVelocityTorquePidGains.hpp
 *
 *  Created on: Oct 27, 2016
 *      Author: Remo Diethelm
 */

#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/ControllerBase.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


class TorqueControllerJointPositionVelocityTorquePidGains : public ControllerBase
{
protected:
  double iError_ = 0.0;

public:
  TorqueControllerJointPositionVelocityTorquePidGains();
  virtual ~TorqueControllerJointPositionVelocityTorquePidGains();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt);
};


} // controller
} // series_elastic_actuator_sim

