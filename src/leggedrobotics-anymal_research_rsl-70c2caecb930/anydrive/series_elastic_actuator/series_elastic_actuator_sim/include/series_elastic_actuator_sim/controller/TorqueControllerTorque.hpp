/*!
 * @file    TorqueControllerTorque.hpp
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


class TorqueControllerTorque: public ControllerBase
{
 public:
  TorqueControllerTorque();
  virtual ~TorqueControllerTorque();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt);
};


} // controller
} // series_elastic_actuator_sim

