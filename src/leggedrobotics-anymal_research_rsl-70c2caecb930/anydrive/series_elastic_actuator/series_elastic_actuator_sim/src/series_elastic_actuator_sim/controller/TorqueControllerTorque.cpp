/*!
 * @file    TorqueControllerTorque.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/TorqueControllerTorque.hpp"


namespace series_elastic_actuator_sim {
namespace controller {


TorqueControllerTorque::TorqueControllerTorque() :
    ControllerBase()
{

}

TorqueControllerTorque::~TorqueControllerTorque()
{

}

bool TorqueControllerTorque::initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt)
{
  return true;
}

bool TorqueControllerTorque::advance(SeActuatorCommand& commandOut, const SeActuatorCommand& commandIn, const SeActuatorState& state, double dt)
{
  double jointTorque = saturate(commandIn.getJointTorque(), commandIn.getJointTorqueMin(), commandIn.getJointTorqueMax());
  commandOut.setJointTorque(jointTorque);
  return true;
}


} // controller
} // series_elastic_actuator_sim
