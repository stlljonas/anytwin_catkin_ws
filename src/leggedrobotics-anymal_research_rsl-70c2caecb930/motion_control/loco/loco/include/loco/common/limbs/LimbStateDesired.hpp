/*
 * LimbStateDesired.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/joints/DesiredJointStates.hpp"
#include "loco/common/typedefs.hpp"

// STL
#include <memory>

namespace loco {
class LimbStateDesired : public DesiredJointStates {
 public:
  explicit LimbStateDesired(const unsigned int numDofLimb);
  ~LimbStateDesired() override = default;
};

using LimbStateDesiredPtr = std::unique_ptr<LimbStateDesired>;

}  // namespace loco
