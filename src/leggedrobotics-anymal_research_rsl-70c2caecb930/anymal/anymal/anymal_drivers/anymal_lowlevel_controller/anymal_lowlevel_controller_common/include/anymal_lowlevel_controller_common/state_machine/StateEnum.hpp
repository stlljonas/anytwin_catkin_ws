#pragma once


// c++
#include <string>

// anymal msgs
#include <anymal_msgs/AnymalLowLevelControllerState.h>


namespace anymal_lowlevel_controller_common {
namespace state_machine {

using StateMsg = anymal_msgs::AnymalLowLevelControllerState;

enum class StateEnum
{
  NA,
  ActionActuatorsClearErrors,
  ActionActuatorsDisable,
  ActionActuatorsEnable,
  ActionActuatorsWarmReset,
  ActionConnectActuators,
  StateFatal,
  StateOperational,
  StateIdle,
  StateZeroJointTorque,
  LAST
};

std::string stateEnumToName(StateEnum stateEnum);
StateEnum stateNameToEnum(const std::string& stateName);

std::string stateEnumToCommandName(StateEnum stateEnum);
StateEnum stateCommandNameToEnum(const std::string& stateName);

StateMsg stateEnumToMsg(StateEnum stateEnum);
StateEnum stateMsgToEnum(const StateMsg& stateMsg);

StateMsg stateNameToMsg(const std::string& stateName);
std::string stateMsgToName(const StateMsg& stateMsg);

bool isAction(StateEnum stateEnum);


} // state_machine
} // anymal_lowlevel_controller_common
