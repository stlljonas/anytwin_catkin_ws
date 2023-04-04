#pragma once


// conditional state machine
#include "conditional_state_machine/Condition.hpp"
#include "conditional_state_machine/HeterogeneousTuple.hpp"


namespace conditional_state_machine {


template <typename... _SubstateTypes>
class Conditions
{
protected:
  using ConditionTupleType = HeterogeneousTuple<Condition<_SubstateTypes>...>;
  const ConditionTupleType conditionTuple_;

public:
  Conditions() {}

  Conditions(Condition<_SubstateTypes>... conditions)
  : conditionTuple_(conditions...) {}

  virtual ~Conditions() {}

  template <typename __SubstatesType>
  bool areFulfilledBy(const __SubstatesType& substates) const
  {
    return conditions_are_fulfilled<__SubstatesType, _SubstateTypes...>::value(conditionTuple_, substates);
  }

protected:
  // Abstract declaration.
  template <typename __SubstatesType, typename... __SubstatesToCheck>
  struct conditions_are_fulfilled;

  // Iterate through conditions.
  template <typename __SubstatesType, typename __FirstSubstate, typename... __OtherSubstates>
  struct conditions_are_fulfilled<__SubstatesType, __FirstSubstate, __OtherSubstates...>
  {
    static bool value(ConditionTupleType conditionTuple, __SubstatesType substates)
    {
      return conditionTuple.template accessElementOfType<Condition<__FirstSubstate>>().isFulfilledBy(substates.template accessElementOfType<__FirstSubstate>()) &&
          conditions_are_fulfilled<__SubstatesType, __OtherSubstates...>::value(conditionTuple, substates);
    }
  };

  // Check last condition.
  template <typename __SubstatesType, typename __LastSubstate>
  struct conditions_are_fulfilled<__SubstatesType, __LastSubstate>
  {
    static bool value(ConditionTupleType conditionTuple, __SubstatesType substates)
    {
      return conditionTuple.template accessElementOfType<Condition<__LastSubstate>>().isFulfilledBy(substates.template accessElementOfType<__LastSubstate>());
    }
  };

  // No conditions are specified (__SubstatesToCheck is empty).
  template <typename __SubstatesType>
  struct conditions_are_fulfilled<__SubstatesType>
  {
    static bool value(ConditionTupleType conditionTuple, __SubstatesType substates)
    {
      return true;
    }
  };
};


} // conditional_state_machine
