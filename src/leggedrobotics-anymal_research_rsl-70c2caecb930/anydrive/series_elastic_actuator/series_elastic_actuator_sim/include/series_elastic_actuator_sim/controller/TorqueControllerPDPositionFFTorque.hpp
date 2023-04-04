/*
 * TorqueControllerPDPositionFFTorque.hpp
 *
 *  Created on: Oct 16, 2015
 *      Author: Georg Wiedebach
 */

#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/ControllerBase.hpp"
#include "series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityPid.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


class TorqueControllerPDPositionFFTorque : public ControllerBase
{
 public:
	TorqueControllerPDPositionFFTorque();
  virtual ~TorqueControllerPDPositionFFTorque();

  virtual bool initialize(
      const SeActuatorCommand& command,
      const SeActuatorState& state,
      double dt);
  virtual bool advance(
      SeActuatorCommand& commandOut,
      const SeActuatorCommand& commandIn,
      const SeActuatorState& state,
      double dt);

  void setPositionGain(double positionGain);
  void setVelocityGain(double velocityGain);

 protected:
  TorqueControllerJointPositionVelocityPid controllerJointPosition_;
};


} // controller
} // series_elastic_actuator_sim
