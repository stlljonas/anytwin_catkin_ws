/*
 * LimbStateDesired.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/common/limbs/LimbStateDesired.hpp"

namespace loco {

LimbStateDesired::LimbStateDesired(const unsigned int numDofLimb) : DesiredJointStates(numDofLimb) {}

}  // namespace loco
