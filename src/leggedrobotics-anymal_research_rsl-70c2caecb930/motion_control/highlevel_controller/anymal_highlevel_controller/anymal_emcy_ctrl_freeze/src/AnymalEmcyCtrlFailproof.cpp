/*!
* @file     AnymalEmcyCtrlFailproof.cpp
* @author   Christian Gehring
* @date     Oct 14, 2011
* @version  1.0
* @ingroup  robotTask
* @brief
*/

#include "anymal_emcy_ctrl_freeze/AnymalEmcyCtrlFailproof.hpp"

#include "rocoma_plugin/rocoma_plugin.hpp"

ROCOMA_EXPORT_FAILPROOF_CONTROLLER(Freeze, anymal_roco::RocoState, anymal_roco::RocoCommand, robot_controller::AnymalEmcyCtrlFailproof)

namespace robot_controller {

AnymalEmcyCtrlFailproof::AnymalEmcyCtrlFailproof()
:Base()
{
  this->setName("freeze");
}

AnymalEmcyCtrlFailproof::~AnymalEmcyCtrlFailproof()
{

}

bool AnymalEmcyCtrlFailproof::create(double dt)
{
  return true;
}

void AnymalEmcyCtrlFailproof::advance(double dt)
{
  for (auto& command : getCommand().getActuatorCommands()) {
    command.setMode(command.Mode::MODE_FREEZE);
    command.setMotorVelocity(0.0);
    command.setJointTorque(0.0);
  }
}

bool AnymalEmcyCtrlFailproof::cleanup() {
  return true;
}

} /* namespace robot_controller */
