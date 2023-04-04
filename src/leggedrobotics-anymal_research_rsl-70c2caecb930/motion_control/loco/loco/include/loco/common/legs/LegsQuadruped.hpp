/*
 * LegsQuadruped.hpp
 *
 *  Created on: Nov, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/GroupContainer.hpp"

// STL
#include <stdexcept>
#include <string>

namespace loco {

//! Container of legs for easy access
template <typename LegType_>
class LegsQuadruped : public GroupContainer<LegType_> {
 public:
  using Base = GroupContainer<LegType_>;
  using ItemSmartPtr = typename Base::ItemSmartPtr;

  //! Constructors
  LegsQuadruped() = default;

  LegsQuadruped(ItemSmartPtr&& leftForeLeg, ItemSmartPtr&& rightForeLeg, ItemSmartPtr&& leftHindLeg, ItemSmartPtr&& rightHindLeg) {
    this->addItem(std::move(leftForeLeg));
    this->addItem(std::move(rightForeLeg));
    this->addItem(std::move(leftHindLeg));
    this->addItem(std::move(rightHindLeg));
  }

  //! Destructor
  ~LegsQuadruped() override = default;

  //! @return const ref to left fore leg
  const LegType_& getLeftForeLeg() const { return *(this->items_[0]); }
  //! @return const ref to right fore leg
  const LegType_& getRightForeLeg() const { return *(this->items_[1]); }
  //! @return const ref to left hind leg
  const LegType_& getLeftHindLeg() const { return *(this->items_[2]); }
  //! @return const ref to right hind leg
  const LegType_& getRightHindLeg() const { return *(this->items_[3]); }

  //! @return ptr to left fore leg
  LegType_* const getLeftForeLegPtr() { return this->items_[0].get(); }
  //! @return ptr to right fore leg
  LegType_* const getRightForeLegPtr() { return this->items_[1].get(); }
  //! @return ptr to left hind leg
  LegType_* const getLeftHindLegPtr() { return this->items_[2].get(); }
  //! @return ptr to right hind leg
  LegType_* const getRightHindLegPtr() { return this->items_[3].get(); }

  /*! Gets leg by identifier
   *
   * @param legId  id of the leg (LF = 0, RF = 1, LH = 2, RH = 3)
   * @return  const ref to leg
   */
  const LegType_& getLegById(unsigned int legId) const {
    for (auto& leg : this->items_) {
      if (leg->getId() == legId) {
        return *leg;
      }
    }
    std::string error = std::string("[LegsQuadruped]: No Leg with ID ") + std::to_string(legId) + std::string("!");
    throw std::out_of_range(error);
  }

  /*! Gets leg by identifier
   *
   * @param legId  id of the leg (LF = 0, RF = 1, LH = 2, RH = 3)
   * @return  ptr to leg
   */
  LegType_* const getLegPtrById(unsigned int legId) {
    for (auto& leg : this->items_) {
      if (leg->getId() == legId) {
        return leg.get();
      }
    }
    return nullptr;
  }
};

} /* namespace loco */
