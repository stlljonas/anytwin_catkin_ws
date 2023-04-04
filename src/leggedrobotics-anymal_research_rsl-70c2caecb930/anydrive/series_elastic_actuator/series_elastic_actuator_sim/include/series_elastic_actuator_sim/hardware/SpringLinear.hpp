/*!
 * @file    SpringLinear.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/hardware/SpringBase.hpp"


namespace series_elastic_actuator_sim {
namespace hardware {


class SpringLinear: public SpringBase
{
 public:
  SpringLinear(double stiffness = 0.0, double damping = 0.0);
  virtual ~SpringLinear();

  virtual bool initialize(const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorState& stateOut, const SeActuatorState& stateIn, double dt);

  double getDamping() const;
  void setDamping(double damping);
  double getStiffness() const;
  void setStiffness(double stiffness);

 protected:
  double stiffness_;
  double damping_;
};


} // hardware
} // series_elastic_actuator_sim

