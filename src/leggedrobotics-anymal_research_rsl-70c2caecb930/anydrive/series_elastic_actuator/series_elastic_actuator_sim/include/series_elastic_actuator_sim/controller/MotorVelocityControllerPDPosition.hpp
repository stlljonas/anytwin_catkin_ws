/*!
 * @file    MotorVelocityControllerPDPosition.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/ControllerBase.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


class MotorVelocityControllerPDPosition: public ControllerBase
{
 public:
  MotorVelocityControllerPDPosition(double positionGain = 0.0, double velocityGain = 0.0);
  virtual ~MotorVelocityControllerPDPosition();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt);

  double getPositionGain() const;
  double getVelocityGain() const;

  void setPositionGain(double positionGain);
  void setVelocityGain(double velocityGain);

 protected:
  double positionGain_;
  double velocityGain_;
};


} // controller
} // series_elastic_actuator_sim

