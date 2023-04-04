#pragma once


// conditional state machine
#include "conditional_state_machine/HomogeneousSet.hpp"
#include "conditional_state_machine/Macros.hpp"


namespace conditional_state_machine {


template <typename _SubstateType>
class Condition
{
protected:
  using SubstateType = _SubstateType;
  using SubstatesType = HomogeneousSet<SubstateType>;
  SubstatesType substates_;
  bool inverted_ = false;

public:
  Condition(const std::set<SubstateType>& substates, bool inverted = false)
  : substates_(substates),
    inverted_(inverted) {}

  bool isFulfilledBy(SubstateType substate)
  {
    const bool isFulfilled = substates_.containsElement(substate) != inverted_;
    if (!isFulfilled)
    {
      CONDITIONAL_STATE_MACHINE_ERROR("Condition is not fulfilled.");
    }
    return isFulfilled;
  }
};


} // conditional_state_machine
