#pragma once


// c++
#include <set>


namespace conditional_state_machine {


template <typename _ElementType>
class HomogeneousSet
{
protected:
  using ElementType = _ElementType;
  std::set<ElementType> set_;

public:
  HomogeneousSet(const std::set<ElementType>& set)
  : set_(set) {}

  virtual ~HomogeneousSet() {}

  bool containsElement(const ElementType& element)
  {
    return set_.find(element) != set_.end();
  }
};


} // conditional_state_machine
