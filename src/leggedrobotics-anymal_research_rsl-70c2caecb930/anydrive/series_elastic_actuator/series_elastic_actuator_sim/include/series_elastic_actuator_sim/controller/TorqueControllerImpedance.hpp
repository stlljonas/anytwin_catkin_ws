/*
 * TorqueControllerImpedance.hpp
 *
 *  Created on: Jun 30, 2015
 *      Author: Dario Bellicoso
 */

#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/ControllerBase.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


class TorqueControllerImpedance : public ControllerBase
{
 public:
  TorqueControllerImpedance(double springGain = 0.0, double damperGain = 0.0, double inertiaGain = 0.0);
  virtual ~TorqueControllerImpedance();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt);

  double getSpringGain() const;
  double getDamperGain() const;
  double getInertiaGain() const;

  void setSpringGain(double springGain);
  void setDamperGain(double damperGain);
  void setInertiaGain(double inertiaGain);

 protected:
  double springGain_;
  double damperGain_;
  double inertiaGain_;
};


} // controller
} // series_elastic_actuator_sim

