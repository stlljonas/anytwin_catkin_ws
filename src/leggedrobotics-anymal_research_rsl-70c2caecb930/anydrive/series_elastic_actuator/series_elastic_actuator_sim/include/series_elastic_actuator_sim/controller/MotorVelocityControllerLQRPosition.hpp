/*!
 * @file    MotorVelocityControllerLQRPosition.hpp
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


class MotorVelocityControllerLQRPosition: public ControllerBase
{
 public:
  MotorVelocityControllerLQRPosition(double jointPositionGain = 0.0, double jointVelocityGain = 0.0, double motorPositionGain = 0.0);
  virtual ~MotorVelocityControllerLQRPosition();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt);

  double getMotorPositionGain() const;
  double getJointPositionGain() const;
  double getJointVelocityGain() const;

  void setMotorPositionGain(double actuatorPositionGain);
  void setJointPositionGain(double jointPositionGain);
  void setJointVelocityGain(double jointVelocityGain);

 protected:
  double jointPositionGain_;
  double jointVelocityGain_;
  double motorPositionGain_;
};


} // controller
} // series_elastic_actuator_sim

