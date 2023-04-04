/*
 * HydraulicLimbStateMeasured.hpp
 *
 *  Created on: Feb 21, 2017
 *      Author: Dominic Jud
 */

#pragma once

// loco
#include "loco/common/joints/MeasuredCylinderStates.hpp"
#include "loco/common/limbs/LimbStateMeasured.hpp"
#include "loco/common/typedefs.hpp"

// STL
#include <memory>

namespace loco {

class HydraulicLimbStateMeasured : public LimbStateMeasured {
 public:
  explicit HydraulicLimbStateMeasured(const unsigned int numDofLimb);
  ~HydraulicLimbStateMeasured() override = default;

  const MeasuredCylinderStates& inCylinderSpace() const;
  MeasuredCylinderStates& inCylinderSpace();

 protected:
  MeasuredCylinderStates measuredCylinderStates_;
};

using HydraulicLimbStateMeasuredPtr = std::unique_ptr<HydraulicLimbStateMeasured>;

} /* namespace loco */
