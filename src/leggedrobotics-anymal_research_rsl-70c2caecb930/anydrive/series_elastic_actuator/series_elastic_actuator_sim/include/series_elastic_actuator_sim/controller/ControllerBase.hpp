/*!
 * @file    ControllerBase.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/usings.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


class ControllerBase
{
 public:
  ControllerBase();
  virtual ~ControllerBase();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt) = 0;
  virtual bool advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt) = 0;
};


} // controller
} // series_elastic_actuator_sim

