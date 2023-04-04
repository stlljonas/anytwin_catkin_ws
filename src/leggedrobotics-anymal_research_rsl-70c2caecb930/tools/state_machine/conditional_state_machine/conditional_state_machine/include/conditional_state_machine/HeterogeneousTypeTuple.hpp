#pragma once


// c++
#include <tuple>


namespace conditional_state_machine {


template <typename... _ElementTypes>
class HeterogeneousTypeTuple
{
protected:
  using TupleType = std::tuple<_ElementTypes...>;

  // Abstract declaration.
  template <typename __Type, typename __Tuple>
  struct set_contains_type;

  // Take this specialization if __Type != __FirstElement.
  template <typename __Type, typename __FirstElement, typename... __OtherElements>
  struct set_contains_type<__Type, std::tuple<__FirstElement, __OtherElements...>> : set_contains_type<__Type, std::tuple<__OtherElements...>> {};

  // Take this specialization if __Type == __FirstElement.
  template <typename __Type, typename... __OtherElements>
  struct set_contains_type<__Type, std::tuple<__Type, __OtherElements...>> : std::true_type {};

  // Take this specialization if __OtherElements is empty.
  template <typename __Type>
  struct set_contains_type<__Type, std::tuple<>> : std::false_type {};

public:
  template <typename __Type>
  using contains_type = typename set_contains_type<__Type, TupleType>::type;
};


} // conditional_state_machine
