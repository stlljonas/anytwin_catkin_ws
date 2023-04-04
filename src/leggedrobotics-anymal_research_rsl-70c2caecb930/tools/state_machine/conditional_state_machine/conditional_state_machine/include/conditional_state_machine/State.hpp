#pragma once


// c++
#include <memory>

// conditional state machine
#include "conditional_state_machine/Conditions.hpp"
#include "conditional_state_machine/Substates.hpp"


namespace conditional_state_machine {


template <typename... _Types>
class State;

template <typename _StateEnumType, typename... _SubstateTypes>
class State<_StateEnumType, Conditions<_SubstateTypes...>>
{
protected:
  using StateEnumType = _StateEnumType;
  using SubstatesType = Substates<_SubstateTypes...>;
  using ConditionsType = Conditions<_SubstateTypes...>;

  const StateEnumType stateEnum_;
  const ConditionsType conditions_;

public:
  State(StateEnumType stateEnum, const ConditionsType& conditions)
  : stateEnum_(stateEnum),
    conditions_(conditions) {}

  virtual ~State() {}

  StateEnumType getStateEnum()
  {
    return stateEnum_;
  }

  bool conditionsAreFulfilledBy(SubstatesType substates)
  {
    return conditions_.areFulfilledBy(substates);
  }

  virtual void enter() {}
  virtual void update() {}
  virtual void leave() {}
};

template <typename... _Types>
using StatePtr = std::shared_ptr<State<_Types...>>;


} // conditional_state_machine
