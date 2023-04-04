/*
 * LimbStateMeasured.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/common/limbs/LimbStateMeasured.hpp"

namespace loco {

LimbStateMeasured::LimbStateMeasured(const unsigned int numDofLimb)
    : MeasuredJointStates(numDofLimb),
      positionWorldToLimbBaseInWorldFrame_(),
      positionWorldToLimbBaseInBaseFrame_(),
      positionBaseToLimbBaseInBaseFrame_(),
      linearVelocityLimbBaseInWorldFrame_() {}

const Position& LimbStateMeasured::getPositionWorldToLimbBaseInBaseFrame() const {
  return positionWorldToLimbBaseInBaseFrame_;
}

void LimbStateMeasured::setPositionWorldToLimbBaseInBaseFrame(const Position& positionWorldToLimbBaseInBaseFrame) {
  positionWorldToLimbBaseInBaseFrame_ = positionWorldToLimbBaseInBaseFrame;
}

const Position& LimbStateMeasured::getPositionBaseToLimbBaseInBaseFrame() const {
  return positionBaseToLimbBaseInBaseFrame_;
}

void LimbStateMeasured::setPositionBaseToLimbBaseInBaseFrame(const Position& positionBaseToLimbBaseInBaseFrame) {
  positionBaseToLimbBaseInBaseFrame_ = positionBaseToLimbBaseInBaseFrame;
}

const Position& LimbStateMeasured::getPositionWorldToLimbBaseInWorldFrame() const {
  return positionWorldToLimbBaseInWorldFrame_;
}

void LimbStateMeasured::setPositionWorldToLimbBaseInWorldFrame(const Position& positionWorldToLimbBaseInWorldFrame) {
  positionWorldToLimbBaseInWorldFrame_ = positionWorldToLimbBaseInWorldFrame;
}

const LinearVelocity& LimbStateMeasured::getLinearVelocityLimbBaseInWorldFrame() const {
  return linearVelocityLimbBaseInWorldFrame_;
}

void LimbStateMeasured::setLinearVelocityLimbBaseInWorldFrame(const LinearVelocity& linearVelocityLimbBaseInWorldFrame) {
  linearVelocityLimbBaseInWorldFrame_ = linearVelocityLimbBaseInWorldFrame;
}

} /* namespace loco */
