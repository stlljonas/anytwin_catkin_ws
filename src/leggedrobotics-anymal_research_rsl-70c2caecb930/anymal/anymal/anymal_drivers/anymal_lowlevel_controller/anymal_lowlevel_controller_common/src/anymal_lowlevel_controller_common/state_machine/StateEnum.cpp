// anymal lowlevel controller
#include "anymal_lowlevel_controller_common/state_machine/StateEnum.hpp"

namespace anymal_lowlevel_controller_common {
namespace state_machine {

std::string stateEnumToName(StateEnum stateEnum) {
  if (stateEnum == StateEnum::ActionActuatorsClearErrors)
    return "ActionActuatorsClearErrors";
  else if (stateEnum == StateEnum::ActionActuatorsDisable)
    return "ActionActuatorsDisable";
  else if (stateEnum == StateEnum::ActionActuatorsEnable)
    return "ActionActuatorsEnable";
  else if (stateEnum == StateEnum::ActionActuatorsWarmReset)
    return "ActionActuatorsWarmReset";
  else if (stateEnum == StateEnum::ActionConnectActuators)
    return "ActionConnectActuators";
  else if (stateEnum == StateEnum::StateFatal)
    return "StateFatal";
  else if (stateEnum == StateEnum::StateOperational)
    return "StateOperational";
  else if (stateEnum == StateEnum::StateIdle)
    return "StateIdle";
  else if (stateEnum == StateEnum::StateZeroJointTorque)
    return "StateZeroJointTorque";
  return "N/A";
}

StateEnum stateNameToEnum(const std::string& stateName) {
  if (stateName == "ActionActuatorsClearErrors")
    return StateEnum::ActionActuatorsClearErrors;
  else if (stateName == "ActionActuatorsDisable")
    return StateEnum::ActionActuatorsDisable;
  else if (stateName == "ActionActuatorsEnable")
    return StateEnum::ActionActuatorsEnable;
  else if (stateName == "ActionActuatorsWarmReset")
    return StateEnum::ActionActuatorsWarmReset;
  else if (stateName == "ActionConnectActuators")
    return StateEnum::ActionConnectActuators;
  else if (stateName == "StateFatal")
    return StateEnum::StateFatal;
  else if (stateName == "StateOperational")
    return StateEnum::StateOperational;
  else if (stateName == "StateIdle")
    return StateEnum::StateIdle;
  else if (stateName == "StateZeroJointTorque")
    return StateEnum::StateZeroJointTorque;
  return StateEnum::NA;
}

std::string stateEnumToCommandName(StateEnum stateEnum) {
  if (stateEnum == StateEnum::ActionActuatorsClearErrors)
    return "Clear Act. Errors";
  else if (stateEnum == StateEnum::ActionActuatorsDisable)
    return "Disable Actuators";
  else if (stateEnum == StateEnum::ActionActuatorsEnable)
    return "Enable Actuators";
  else if (stateEnum == StateEnum::ActionActuatorsWarmReset)
    return "Warm Reset Act.";
  else if (stateEnum == StateEnum::ActionConnectActuators)
    return "Connect Actuators";
  else if (stateEnum == StateEnum::StateFatal)
    return "Enter Fatal State";
  else if (stateEnum == StateEnum::StateOperational)
    return "Enter Operational State";
  else if (stateEnum == StateEnum::StateIdle)
    return "Enter Idle State";
  else if (stateEnum == StateEnum::StateZeroJointTorque)
    return "Enter Zero Torque State";
  return "N/A";
}

StateEnum stateCommandNameToEnum(const std::string& stateName) {
  if (stateName == "Clear Act. Errors")
    return StateEnum::ActionActuatorsClearErrors;
  else if (stateName == "Disable Actuators")
    return StateEnum::ActionActuatorsDisable;
  else if (stateName == "Enable Actuators")
    return StateEnum::ActionActuatorsEnable;
  else if (stateName == "Warm Reset Act.")
    return StateEnum::ActionActuatorsWarmReset;
  else if (stateName == "Connect Actuators")
    return StateEnum::ActionConnectActuators;
  else if (stateName == "Enter Fatal State")
    return StateEnum::StateFatal;
  else if (stateName == "Enter Operational State")
    return StateEnum::StateOperational;
  else if (stateName == "Enter Idle State")
    return StateEnum::StateIdle;
  else if (stateName == "Enter Zero Torque State")
    return StateEnum::StateZeroJointTorque;
  return StateEnum::NA;
}

StateMsg stateEnumToMsg(StateEnum stateEnum) {
  StateMsg stateMsg;
  if (stateEnum == StateEnum::ActionActuatorsClearErrors)
    stateMsg.state = StateMsg::ACTION_ACTUATORS_CLEAR_ERRORS;
  else if (stateEnum == StateEnum::ActionActuatorsDisable)
    stateMsg.state = StateMsg::ACTION_ACTUATORS_DISABLE;
  else if (stateEnum == StateEnum::ActionActuatorsEnable)
    stateMsg.state = StateMsg::ACTION_ACTUATORS_ENABLE;
  else if (stateEnum == StateEnum::ActionActuatorsWarmReset)
    stateMsg.state = StateMsg::ACTION_ACTUATORS_WARM_RESET;
  else if (stateEnum == StateEnum::ActionConnectActuators)
    stateMsg.state = StateMsg::ACTION_CONNECT_ACTUATORS;
  else if (stateEnum == StateEnum::StateFatal)
    stateMsg.state = StateMsg::STATE_FATAL;
  else if (stateEnum == StateEnum::StateOperational)
    stateMsg.state = StateMsg::STATE_OPERATIONAL;
  else if (stateEnum == StateEnum::StateIdle)
    stateMsg.state = StateMsg::STATE_IDLE;
  else if (stateEnum == StateEnum::StateZeroJointTorque)
    stateMsg.state = StateMsg::STATE_ZERO_JOINT_TORQUE;
  return stateMsg;
}

StateEnum stateMsgToEnum(const StateMsg& stateMsg) {
  if (stateMsg.state == StateMsg::ACTION_ACTUATORS_CLEAR_ERRORS)
    return StateEnum::ActionActuatorsClearErrors;
  else if (stateMsg.state == StateMsg::ACTION_ACTUATORS_DISABLE)
    return StateEnum::ActionActuatorsDisable;
  else if (stateMsg.state == StateMsg::ACTION_ACTUATORS_ENABLE)
    return StateEnum::ActionActuatorsEnable;
  else if (stateMsg.state == StateMsg::ACTION_ACTUATORS_WARM_RESET)
    return StateEnum::ActionActuatorsWarmReset;
  else if (stateMsg.state == StateMsg::ACTION_CONNECT_ACTUATORS)
    return StateEnum::ActionConnectActuators;
  else if (stateMsg.state == StateMsg::STATE_FATAL)
    return StateEnum::StateFatal;
  else if (stateMsg.state == StateMsg::STATE_OPERATIONAL)
    return StateEnum::StateOperational;
  else if (stateMsg.state == StateMsg::STATE_IDLE)
    return StateEnum::StateIdle;
  else if (stateMsg.state == StateMsg::STATE_ZERO_JOINT_TORQUE)
    return StateEnum::StateZeroJointTorque;
  return StateEnum::NA;
}

StateMsg stateNameToMsg(const std::string& stateName) {
  return stateEnumToMsg(stateNameToEnum(stateName));
}

std::string stateMsgToName(const StateMsg& stateMsg) {
  return stateEnumToName(stateMsgToEnum(stateMsg));
}

bool isAction(StateEnum stateEnum) {
  return (stateEnum == StateEnum::ActionActuatorsClearErrors || stateEnum == StateEnum::ActionActuatorsDisable ||
          stateEnum == StateEnum::ActionActuatorsEnable || stateEnum == StateEnum::ActionActuatorsWarmReset ||
          stateEnum == StateEnum::ActionConnectActuators);
}

}  // namespace state_machine
}  // namespace anymal_lowlevel_controller_common
