/*!
 * @file    SingleArmGroup.hpp
 * @author  Gabriel Hottiger
 * @date    Nov, 2017
 */

#pragma once

// loco
#include "loco/common/GroupContainer.hpp"

namespace loco {

//! Container of arms for easy access
template <typename ArmType_>
class SingleArmGroup : public loco::GroupContainer<ArmType_> {
 public:
  using Base = loco::GroupContainer<ArmType_>;
  using ItemSmartPtr = typename Base::ItemSmartPtr;

  //! Constructor
  SingleArmGroup() = default;

  explicit SingleArmGroup(ItemSmartPtr&& arm) { this->addItem(std::move(arm)); }

  //! Destructor
  virtual ~SingleArmGroup() = default;

  //! @return const ref to arm
  const ArmType_& getArm() const { return *(this->items_[0]); }

  //! @return ptr to arm
  ArmType_* const getArmPtr() { return this->items_[0].get(); }
};

} /* namespace loco */
