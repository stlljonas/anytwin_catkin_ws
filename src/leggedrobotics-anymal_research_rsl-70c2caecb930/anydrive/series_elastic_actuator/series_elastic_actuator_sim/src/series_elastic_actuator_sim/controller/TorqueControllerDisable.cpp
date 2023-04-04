/*
 * TorqueControllerDisable.cpp
 *
 *  Created on: Oct 15, 2015
 *      Author: dbellicoso
 */

// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/TorqueControllerDisable.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


TorqueControllerDisable::TorqueControllerDisable()
    : ControllerBase()
{

}

TorqueControllerDisable::~TorqueControllerDisable()
{

}

bool TorqueControllerDisable::initialize(const SeActuatorCommand& command,
                                         const SeActuatorState& state, double dt)
{
  return true;
}

bool TorqueControllerDisable::advance(SeActuatorCommand& commandOut,
                                      const SeActuatorCommand& commandIn,
                                      const SeActuatorState& state, double dt)
{
  commandOut.setJointTorque(0.0);
  return true;
}


} // controller
} // series_elastic_actuator_sim
