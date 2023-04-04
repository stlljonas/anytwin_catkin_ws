/*
 * TorqueControllerDisable.hpp
 *
 *  Created on: Oct 15, 2015
 *      Author: Dario Bellicoso
 */

#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/ControllerBase.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


class TorqueControllerDisable : public ControllerBase
{
 public:
  TorqueControllerDisable();
  virtual ~TorqueControllerDisable();

  virtual bool initialize(
      const SeActuatorCommand& command,
      const SeActuatorState& state,
      double dt);
  virtual bool advance(
      SeActuatorCommand& commandOut,
      const SeActuatorCommand& commandIn,
      const SeActuatorState& state,
      double dt);
};


} // controller
} // series_elastic_actuator_sim

