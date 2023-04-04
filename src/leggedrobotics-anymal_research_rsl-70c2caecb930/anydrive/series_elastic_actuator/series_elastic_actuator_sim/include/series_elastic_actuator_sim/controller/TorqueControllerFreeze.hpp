/*!
 * @file    TorqueControllerFreeze.hpp
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


class TorqueControllerFreeze: public ControllerBase
{
 public:
  TorqueControllerFreeze(double positionGain = 0.0, double velocityGain = 0.0);
  virtual ~TorqueControllerFreeze();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt);

  double getJointPositionPGain() const;
  double getJointPositionDGain() const;

  void setJointPositionPGain(double positionGain);
  void setJointPositionDGain(double velocityGain);

 protected:
  double positionGain_;
  double velocityGain_;

  double jointPositionDes_ = 0.0;
};


} // controller
} // series_elastic_actuator_sim

