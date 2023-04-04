#pragma once


// message logger
#include <message_logger/message_logger.hpp>

// any measurements
#include <any_measurements/Time.hpp>

// conditional state machine
#include <conditional_state_machine/conditional_state_machine.hpp>

// anymal lowlevel controller common
#include "anymal_lowlevel_controller_common/state_machine/StateEnum.hpp"

// anymal lowlevel controller
#include "anymal_lowlevel_controller/state_machine/substates.hpp"
#include "anymal_lowlevel_controller/System.hpp"


namespace anymal_lowlevel_controller {
namespace state_machine {

using StateEnum = anymal_lowlevel_controller_common::state_machine::StateEnum;
using anymal_lowlevel_controller_common::state_machine::stateEnumToMsg;
using anymal_lowlevel_controller_common::state_machine::stateMsgToEnum;
using anymal_lowlevel_controller_common::state_machine::isAction;


class StateMachine : public conditional_state_machine::StateMachine<StateEnum, Substates>
{
protected:
  using Base = conditional_state_machine::StateMachine<StateEnum, Substates>;
  using States = Base::StatesType;

  SystemPtr system_;

public:
  StateMachine(const SystemPtr& system);
  virtual ~StateMachine();

  const SystemPtr& getSystem();
};

using StateMachinePtr = std::shared_ptr<StateMachine>;


} // state_machine
} // anymal_lowlevel_controller
