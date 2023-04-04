/*
 * WholeBodyStateDesired.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: Dario Bellicoso
 */

#include <loco/common/WholeBodyStateDesired.hpp>

namespace loco {

WholeBodyStateDesired::WholeBodyStateDesired()
    : positionWorldToWholeBodyCenterOfMassInWorldFrame_(),
      linearVelocityWholeBodyCenterOfMassInWorldFrame_(),
      linearAccelerationWholeBodyCenterOfMassInWorldFrame_() {}

const Position& WholeBodyStateDesired::getPositionWorldToWholeBodyCenterOfMassInWorldFrame() const {
  return positionWorldToWholeBodyCenterOfMassInWorldFrame_;
}

void WholeBodyStateDesired::setPositionWorldToWholeBodyCenterOfMassInWorldFrame(
    const Position& positionWorldToWholeBodyCenterOfMassInWorldFrame) {
  positionWorldToWholeBodyCenterOfMassInWorldFrame_ = positionWorldToWholeBodyCenterOfMassInWorldFrame;
}

const LinearVelocity& WholeBodyStateDesired::getLinearVelocityWholeBodyCenterOfMassInWorldFrame() const {
  return linearVelocityWholeBodyCenterOfMassInWorldFrame_;
}

void WholeBodyStateDesired::setLinearVelocityWholeBodyCenterOfMassInWorldFrame(
    const LinearVelocity& linearVelocityWholeBodyCenterOfMassInWorldFrame) {
  linearVelocityWholeBodyCenterOfMassInWorldFrame_ = linearVelocityWholeBodyCenterOfMassInWorldFrame;
}

const LinearAcceleration& WholeBodyStateDesired::getLinearAccelerationWholeBodyCenterOfMassInWorldFrame() const {
  return linearAccelerationWholeBodyCenterOfMassInWorldFrame_;
}

void WholeBodyStateDesired::setLinearAccelerationWholeBodyCenterOfMassInWorldFrame(
    const LinearAcceleration& linearAccelerationWholeBodyCenterOfMassInWorldFrame) {
  linearAccelerationWholeBodyCenterOfMassInWorldFrame_ = linearAccelerationWholeBodyCenterOfMassInWorldFrame;
}

} /* namespace loco */
