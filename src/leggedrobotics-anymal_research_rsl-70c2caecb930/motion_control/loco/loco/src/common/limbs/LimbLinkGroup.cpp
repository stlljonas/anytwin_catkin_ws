/*
 * LegLinkGroup.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

// loco
#include "loco/common/limbs/LimbLinkGroup.hpp"

// STL
#include <exception>

namespace loco {

const LimbLink& LimbLinkGroup::getLimbLinkById(unsigned int linkId) const {
  for (auto& link : items_) {
    if (link->getLinkId() == linkId) {
      return *(link.get());
    }
  }
  std::string error = std::string("[LimbLinkGroup]: No limb link with ID ") + std::to_string(linkId) + std::string("!");
  throw std::out_of_range(error);
}

LimbLink* LimbLinkGroup::getLimbLinkPtrById(unsigned int linkId) {
  for (auto& link : items_) {
    if (link->getLinkId() == linkId) {
      return link.get();
    }
  }
  return nullptr;
}

} /* namespace loco */
