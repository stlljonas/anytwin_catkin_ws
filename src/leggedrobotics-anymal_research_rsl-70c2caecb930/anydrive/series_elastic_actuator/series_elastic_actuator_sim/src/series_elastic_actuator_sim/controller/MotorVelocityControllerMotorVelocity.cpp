/*!
 * @file    MotorVelocityControllerMotorVelocity.cpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/MotorVelocityControllerMotorVelocity.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


MotorVelocityControllerMotorVelocity::MotorVelocityControllerMotorVelocity() :
    ControllerBase()
{

}

MotorVelocityControllerMotorVelocity::~MotorVelocityControllerMotorVelocity()
{

}

bool MotorVelocityControllerMotorVelocity::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt)
{
  return true;
}

bool MotorVelocityControllerMotorVelocity::advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt)
{
  double motorVelocity = saturate(commandIn.getMotorVelocity(), commandIn.getMotorVelocityMin(), commandIn.getMotorVelocityMax());
  commandOut.setMotorVelocity(motorVelocity);
  return true;
}


} // controller
} // series_elastic_actuator_sim
