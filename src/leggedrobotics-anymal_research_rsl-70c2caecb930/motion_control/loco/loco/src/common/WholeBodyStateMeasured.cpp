/*
 * WholeBodyStateMeasured.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: Dario Bellicoso
 */

#include <loco/common/WholeBodyStateMeasured.hpp>

namespace loco {

WholeBodyStateMeasured::WholeBodyStateMeasured()
    : positionWorldToWholeBodyCenterOfMassInWorldFrame_(),
      linearVelocityWholeBodyCenterOfMassInWorldFrame_(),
      linearAccelerationWholeBodyCenterOfMassInWorldFrame_(),
      positionControlToWholeBodyCenterOfMassInControlFrame_(),
      linearVelocityWholeBodyCenterOfMassInControlFrame_() {}

const Position& WholeBodyStateMeasured::getPositionWorldToWholeBodyCenterOfMassInWorldFrame() const {
  return positionWorldToWholeBodyCenterOfMassInWorldFrame_;
}

void WholeBodyStateMeasured::setPositionWorldToWholeBodyCenterOfMassInWorldFrame(
    const Position& positionWorldToWholeBodyCenterOfMassInWorldFrame) {
  positionWorldToWholeBodyCenterOfMassInWorldFrame_ = positionWorldToWholeBodyCenterOfMassInWorldFrame;
}

const LinearVelocity& WholeBodyStateMeasured::getLinearVelocityWholeBodyCenterOfMassInWorldFrame() const {
  return linearVelocityWholeBodyCenterOfMassInWorldFrame_;
}

void WholeBodyStateMeasured::setLinearVelocityWholeBodyCenterOfMassInWorldFrame(
    const LinearVelocity& linearVelocityWholeBodyCenterOfMassInWorldFrame) {
  linearVelocityWholeBodyCenterOfMassInWorldFrame_ = linearVelocityWholeBodyCenterOfMassInWorldFrame;
}

const Position& WholeBodyStateMeasured::getPositionControlToWholeBodyCenterOfMassInControlFrame() const {
  return positionControlToWholeBodyCenterOfMassInControlFrame_;
}

void WholeBodyStateMeasured::setPositionControlToWholeBodyCenterOfMassInControlFrame(
    const Position& positionControlToWholeBodyCenterOfMassInControlFrame) {
  positionControlToWholeBodyCenterOfMassInControlFrame_ = positionControlToWholeBodyCenterOfMassInControlFrame;
}

const Position& WholeBodyStateMeasured::getPositionWorldToCenterOfPressureInWorldFrame() const {
  return positionWorldToCenterOfPressureInWorldFrame_;
}

void WholeBodyStateMeasured::setPositionWorldToCenterOfPressureInWorldFrame(const Position& positionWorldToCenterOfPressureInWorldFrame) {
  positionWorldToCenterOfPressureInWorldFrame_ = positionWorldToCenterOfPressureInWorldFrame;
}

const LinearVelocity& WholeBodyStateMeasured::getLinearVelocityWholeBodyCenterOfMassInControlFrame() const {
  return linearVelocityWholeBodyCenterOfMassInControlFrame_;
}

void WholeBodyStateMeasured::setLinearVelocityWholeBodyCenterOfMassInControlFrame(
    const LinearVelocity& linearVelocityWholeBodyCenterOfMassInControlFrame) {
  linearVelocityWholeBodyCenterOfMassInControlFrame_ = linearVelocityWholeBodyCenterOfMassInControlFrame;
}

const Position& WholeBodyStateMeasured::getPositionWorldToDCMInWorldFrame() const {
  return positionWorldToDCMInWorldFrame_;
}

void WholeBodyStateMeasured::updatePositionWorldToDCMInWorldFrame(double omega) {
  const Position& comPos = positionWorldToWholeBodyCenterOfMassInWorldFrame_;
  const LinearVelocity& comVel = linearVelocityWholeBodyCenterOfMassInWorldFrame_;
  positionWorldToDCMInWorldFrame_ = comPos + Position(comVel / omega);
}

} /* namespace loco */
