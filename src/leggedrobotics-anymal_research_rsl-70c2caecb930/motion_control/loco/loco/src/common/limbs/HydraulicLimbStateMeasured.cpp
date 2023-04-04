/*
 * HydraulicLimbStateMeasured.cpp
 *
 *  Created on: Feb 21, 2017
 *      Author: Dominic Jud
 */

// loco
#include "loco/common/limbs/HydraulicLimbStateMeasured.hpp"

namespace loco {

HydraulicLimbStateMeasured::HydraulicLimbStateMeasured(const unsigned int numDofLimb)
    : LimbStateMeasured(numDofLimb), measuredCylinderStates_(numDofLimb) {}

const MeasuredCylinderStates& HydraulicLimbStateMeasured::inCylinderSpace() const {
  return measuredCylinderStates_;
}

MeasuredCylinderStates& HydraulicLimbStateMeasured::inCylinderSpace() {
  return measuredCylinderStates_;
}

} /* namespace loco */
