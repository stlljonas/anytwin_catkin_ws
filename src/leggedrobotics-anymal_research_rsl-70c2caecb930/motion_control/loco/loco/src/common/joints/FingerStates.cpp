/*
 * FingerStates.cpp
 *
 *  Created on: Jul, 2018
 *      Author: Markus Staeuble
 */

#include "loco/common/joints/FingerStates.hpp"

// STL
#include <cassert>

namespace loco {

FingerStates::FingerStates(unsigned int numFingers) : numFingers_(numFingers) {
  fingerPositions_.resize(numFingers_, 1);
  fingerPositions_.setZero();
  fingerVelocities_.resize(numFingers_, 1);
  fingerVelocities_.setZero();
}

const JointPositions& FingerStates::getFingerPositions() const {
  return fingerPositions_;
}

void FingerStates::setFingerPositions(const JointPositions& fingerPositions) {
  assert(fingerPositions.rows() == fingerPositions_.rows());
  fingerPositions_ = fingerPositions;
}

void FingerStates::setFingerPosition(const unsigned int index, const JointPosition& fingerPosition) {
  assert(index < fingerPositions_.rows());
  fingerPositions_(index) = fingerPosition;
}

const JointPositions& FingerStates::getFingerVelocities() const {
  return fingerVelocities_;
}

void FingerStates::setFingerVelocities(const JointPositions& fingerVelocities) {
  assert(fingerVelocities.rows() == fingerVelocities_.rows());
  fingerVelocities_ = fingerVelocities;
}

void FingerStates::setFingerVelocity(const unsigned int index, const JointPosition& fingerVelocity) {
  assert(index < fingerVelocities_.rows());
  fingerVelocities_(index) = fingerVelocity;
}

} /* namespace loco */