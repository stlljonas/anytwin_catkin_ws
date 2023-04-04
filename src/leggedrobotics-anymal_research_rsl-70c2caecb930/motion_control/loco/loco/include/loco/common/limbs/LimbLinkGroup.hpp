/*
 * LegLinkGroup.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/GroupContainer.hpp"
#include "loco/common/limbs/LimbLink.hpp"

// stl
#include <memory>

namespace loco {

class LimbLinkGroup : public loco::GroupContainer<LimbLink> {
 public:
  //! Constructor
  LimbLinkGroup() = default;

  //! Destructor
  ~LimbLinkGroup() override = default;

  /*! Gets limb links by identifier
   * @param linkId
   * @return  const ref to leg
   */
  const LimbLink& getLimbLinkById(unsigned int linkId) const;

  /*! Gets limb links by identifier
   * @param linkId
   * @return  ptr to limb link
   */
  LimbLink* getLimbLinkPtrById(unsigned int linkId);
};

} /* namespace loco */
