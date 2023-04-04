/*!
 * @file    SpringBase.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/usings.hpp"


namespace series_elastic_actuator_sim {
namespace hardware {


class SpringBase
{
 public:
  SpringBase();
  virtual ~SpringBase();

  virtual bool initialize(const SeActuatorState& state, double dt) = 0;
  virtual bool advance(SeActuatorState& stateOut, const SeActuatorState& stateIn, double dt) = 0;
};


} // hardware
} // series_elastic_actuator_sim

