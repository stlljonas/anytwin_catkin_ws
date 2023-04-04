#pragma once


// conditional state machine
#include "conditional_state_machine/HeterogeneousTuple.hpp"


namespace conditional_state_machine {


template <typename... _SubstateTypes>
class Substates : public HeterogeneousTuple<_SubstateTypes...>
{
protected:
  using BaseType = HeterogeneousTuple<_SubstateTypes...>;
  using typename BaseType::TupleType;

public:
  Substates()
  : BaseType() {}

  Substates(_SubstateTypes... substates)
  : BaseType(substates...) {}

  virtual ~Substates() {}
};


} // conditional_state_machine
