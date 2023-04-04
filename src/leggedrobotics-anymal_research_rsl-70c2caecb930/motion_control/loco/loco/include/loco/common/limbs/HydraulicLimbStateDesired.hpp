/*
 * HydraulicLimbStateDesired.hpp
 *
 *  Created on: Jan 4, 2018
 *      Author: Dominic Jud
 */

#pragma once

// loco
#include "loco/common/limbs/LimbStateDesired.hpp"
#include "loco/common/typedefs.hpp"

// STL
#include <memory>

namespace loco {

template <typename T>
class HydraulicLimbStateDesired : public LimbStateDesired {
 public:
  HydraulicLimbStateDesired(const unsigned int numDofLimb) : LimbStateDesired(numDofLimb) { valves_.resize(numDofLimb); };
  virtual ~HydraulicLimbStateDesired() = default;

  const std::vector<T>& getValves() const { return valves_; };
  std::vector<T>& getValves() { return valves_; };

 protected:
  std::vector<T> valves_;
};

template <typename T>
using HydraulicLimbStateDesiredPtr = std::unique_ptr<HydraulicLimbStateDesired<T>>;

} /* namespace loco */
