/*!
 * @file    MotorVelocityControllerTorque.hpp
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


class MotorVelocityControllerTorque : public ControllerBase
{
 public:
  MotorVelocityControllerTorque(double torqueGain = 0.0);
  virtual ~MotorVelocityControllerTorque();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt);

  double getTorqueGain() const;
  void setTorqueGain(double torqueGain);

 protected:
  double torqueGain_;
};


} // controller
} // series_elastic_actuator_sim

