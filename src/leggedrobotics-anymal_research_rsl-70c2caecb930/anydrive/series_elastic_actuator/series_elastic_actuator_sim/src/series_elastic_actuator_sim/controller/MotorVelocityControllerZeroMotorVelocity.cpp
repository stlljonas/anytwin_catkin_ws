/*!
 * @file    MotorVelocityControllerZeroMotorVelocity.cpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/MotorVelocityControllerZeroMotorVelocity.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


MotorVelocityControllerZeroMotorVelocity::MotorVelocityControllerZeroMotorVelocity() :
    ControllerBase()
{

}

MotorVelocityControllerZeroMotorVelocity::~MotorVelocityControllerZeroMotorVelocity()
{

}

bool MotorVelocityControllerZeroMotorVelocity::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt)
{
  return true;
}

bool MotorVelocityControllerZeroMotorVelocity::advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt)
{
  commandOut.setMotorVelocity(0.0);
  return true;
}


} // controller
} // series_elastic_actuator_sim
