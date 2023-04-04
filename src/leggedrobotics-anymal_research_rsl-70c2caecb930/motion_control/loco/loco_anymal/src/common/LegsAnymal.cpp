/*
 * LegGroup.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#include "loco_anymal/common/LegsAnymal.hpp"

// STL
#include <exception>

namespace loco_anymal {

LegsAnymal::LegsAnymal(
    ItemSmartPtr&& leftForeLeg,
    ItemSmartPtr&& rightForeLeg,
    ItemSmartPtr&& leftHindLeg,
    ItemSmartPtr&& rightHindLeg)
{
  addItem(std::move(leftForeLeg));
  addItem(std::move(rightForeLeg));
  addItem(std::move(leftHindLeg));
  addItem(std::move(rightHindLeg));
}

const LegAnymal& LegsAnymal::getLegById(int legId) const {
  for (auto& leg : items_) {
    if (leg->getId() == static_cast<unsigned>(legId)) {
      return *leg;
    }
  }
  std::string error = std::string("[LegsAnymal]: No Leg with ID ") + std::to_string(legId) + std::string("!");
  throw std::out_of_range(error);
}

LegAnymal* LegsAnymal::getLegPtrById(int legId) {
  for (auto& leg : items_) {
    if (leg->getId() == static_cast<unsigned>(legId)) {
      return leg.get();
    }
  }
  return nullptr;
}

} /* namespace loco_anymal */
