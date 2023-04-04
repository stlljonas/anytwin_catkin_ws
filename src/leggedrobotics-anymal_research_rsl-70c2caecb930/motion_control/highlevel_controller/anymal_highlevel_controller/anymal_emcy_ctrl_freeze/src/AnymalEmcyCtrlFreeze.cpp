/*!
* @file     AnymalEmcyCtrlFreeze.cpp
* @author   Christian Gehring
* @date     Oct 14, 2011
* @version  1.0
* @ingroup  robotTask
* @brief
*/

#include "anymal_emcy_ctrl_freeze/AnymalEmcyCtrlFreeze.hpp"

// rocoma plugin
#include "rocoma_plugin/rocoma_plugin.hpp"

ROCOMA_EXPORT_CONTROLLER(Freeze, anymal_roco::RocoState, anymal_roco::RocoCommand, robot_controller::AnymalEmcyCtrlFreeze)
ROCOMA_EXPORT_EMERGENCY_CONTROLLER(Freeze, anymal_roco::RocoState, anymal_roco::RocoCommand, robot_controller::AnymalEmcyCtrlFreeze)


namespace robot_controller {

AnymalEmcyCtrlFreeze::AnymalEmcyCtrlFreeze()
:Base()
{
  this->setName("freeze");
}

AnymalEmcyCtrlFreeze::~AnymalEmcyCtrlFreeze()
{

}

bool AnymalEmcyCtrlFreeze::initializeFast(double dt) {
  for (auto& command : getCommand().getActuatorCommands()) {
    command.setMode(command.Mode::MODE_FREEZE);
    command.setMotorVelocity(0.0);
    command.setJointVelocity(0.0);
    command.setJointTorque(0.0);
  }
  return true;
}

bool AnymalEmcyCtrlFreeze::advance(double dt)
{
  for (auto& command : getCommand().getActuatorCommands()) {
    command.setMode(command.Mode::MODE_FREEZE);
    command.setMotorVelocity(0.0);
    command.setJointVelocity(0.0);
    command.setJointTorque(0.0);
  }
  return true;
}

} /* namespace robot_controller */
