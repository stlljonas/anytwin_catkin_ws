#pragma once


// conditional state machine
#include "conditional_state_machine/HomogeneousSet.hpp"
#include "conditional_state_machine/Macros.hpp"
#include "conditional_state_machine/State.hpp"


namespace conditional_state_machine {


template <typename... _Types>
class States;

template <typename _StateEnumType, typename... _SubstateTypes>
class States<_StateEnumType, Conditions<_SubstateTypes...>> : public HomogeneousSet<StatePtr<_StateEnumType, Conditions<_SubstateTypes...>>>
{
protected:
  using StateEnumType = _StateEnumType;
  using StatePtrType = StatePtr<StateEnumType, Conditions<_SubstateTypes...>>;
  using BaseType = HomogeneousSet<StatePtrType>;

public:
  States(const std::set<StatePtrType>& states)
  : BaseType(states) {}

  virtual ~States() {}

  StatePtrType getState(StateEnumType stateEnum)
  {
    for (auto state : this->set_)
      if (state->getStateEnum() == stateEnum)
        return state;

    CONDITIONAL_STATE_MACHINE_ERROR("The state has not been found.");
    return StatePtrType();
  }
};


} // conditional_state_machine
