#pragma once


// conditional state machine
#include "conditional_state_machine/HeterogeneousTypeTuple.hpp"


namespace conditional_state_machine {


template <typename... _ElementTypes>
class HeterogeneousTuple : public HeterogeneousTypeTuple<_ElementTypes...>
{
protected:
  using typename HeterogeneousTypeTuple<_ElementTypes...>::TupleType;
  TupleType tuple_;

public:
  HeterogeneousTuple() {}

  HeterogeneousTuple(_ElementTypes... elements)
  : tuple_(std::make_tuple(elements...)) {}

  virtual ~HeterogeneousTuple() {}

  template <typename __Type>
  __Type& accessElementOfType()
  {
    return get_matching_element<0, __Type, TupleType, ElementIsOfType<0, __Type>::value>::get(tuple_);
  }

protected:
  // Abstract declaration.
  template<int __Index, typename __Type>
  struct ElementIsOfType : std::is_same<__Type, typename std::tuple_element<__Index, TupleType>::type> {};

  // Take this specialization if __Match != true.
  template <int __Index, class __Type, class __Tuple, bool __Match>
  struct get_matching_element
  {
    static __Type& get(__Tuple& tuple)
    {
      return get_matching_element<__Index+1, __Type, __Tuple, ElementIsOfType<__Index+1, __Type>::value>::get(tuple);
    }
  };

  // Take this specialization if __Match == true.
  template <int __Index, class __Type, class __Tuple>
  struct get_matching_element<__Index, __Type, __Tuple, true>
  {
    static __Type& get(__Tuple& tuple)
    {
      return std::get<__Index>(tuple);
    }
  };
};


} // conditional_state_machine
