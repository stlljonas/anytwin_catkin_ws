/*
 * EndEffectorStateDesired.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

// loco
#include <loco/common/end_effectors/EndEffectorStateDesired.hpp>

namespace loco {

EndEffectorStateDesired::EndEffectorStateDesired() : EndEffectorStateBase(), desiredJointStatesLimb_(nullptr) {}

void EndEffectorStateDesired::setDesiredJointStatesLimb(DesiredJointStates* const desiredJointStatesLimb) {
  desiredJointStatesLimb_ = desiredJointStatesLimb;
}

} /* namespace loco */
