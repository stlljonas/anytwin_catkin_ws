/*
 * TorsoStateMeasured.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring
 */

#include "loco/common/torso/TorsoStateMeasured.hpp"

namespace loco {

TorsoStateMeasured::TorsoStateMeasured()
    : stateInControlFrame_(),
      positionWorldToBaseInWorldFrame_(),
      positionWorldToCenterOfMassInWorldFrame_(),
      orientationWorldToBase_(),
      linearVelocityBaseInBaseFrame_(),
      linearVelocityCenterOfMassInBaseFrame_(),
      angularVelocityBaseInBaseFrame_() {}

const loco::Position& TorsoStateMeasured::getPositionWorldToBaseInWorldFrame() const {
  return positionWorldToBaseInWorldFrame_;
}

const RotationQuaternion& TorsoStateMeasured::getOrientationWorldToBase() const {
  return orientationWorldToBase_;
}

void TorsoStateMeasured::setOrientationWorldToBase(const RotationQuaternion& orientation) {
  orientationWorldToBase_ = orientation;
  orientationEulerAnglesZyxBaseToWorld_ = EulerAnglesZyx(orientation.inverted());
  orientationEulerAnglesZyxBaseToWorld_.setUnique();
}

const loco::LinearVelocity& TorsoStateMeasured::getLinearVelocityBaseInBaseFrame() const {
  return linearVelocityBaseInBaseFrame_;
}

void TorsoStateMeasured::setLinearVelocityBaseInBaseFrame(const LinearVelocity& linearVelocity) {
  linearVelocityBaseInBaseFrame_ = linearVelocity;
}

const LinearVelocity& TorsoStateMeasured::getLinearVelocityCenterOfMassInBaseFrame() const {
  return linearVelocityCenterOfMassInBaseFrame_;
}

void TorsoStateMeasured::setLinearVelocityCenterOfMassInBaseFrame(const LinearVelocity& linearVelocityCenterOfMassInBaseFrame) {
  linearVelocityCenterOfMassInBaseFrame_ = linearVelocityCenterOfMassInBaseFrame;
}

const loco::LocalAngularVelocity& TorsoStateMeasured::getAngularVelocityBaseInBaseFrame() const {
  return angularVelocityBaseInBaseFrame_;
}

void TorsoStateMeasured::setAngularVelocityBaseInBaseFrame(const loco::LocalAngularVelocity& angularVelocity) {
  angularVelocityBaseInBaseFrame_ = angularVelocity;
}

void TorsoStateMeasured::setPositionWorldToBaseInWorldFrame(const Position& position) {
  positionWorldToBaseInWorldFrame_ = position;
}

const EulerAnglesZyx& TorsoStateMeasured::getOrientationEulerAnglesZyxBaseToWorld() const {
  return orientationEulerAnglesZyxBaseToWorld_;
}

const Position& TorsoStateMeasured::getPositionWorldToCenterOfMassInWorldFrame() const {
  return positionWorldToCenterOfMassInWorldFrame_;
}

void TorsoStateMeasured::setPositionWorldToCenterOfMassInWorldFrame(const Position& positionWorldToCenterOfMassInWorldFrame) {
  positionWorldToCenterOfMassInWorldFrame_ = positionWorldToCenterOfMassInWorldFrame;
}

const TorsoStateMeasuredInControlFrame& TorsoStateMeasured::inControlFrame() const {
  return stateInControlFrame_;
}

TorsoStateMeasuredInControlFrame& TorsoStateMeasured::inControlFrame() {
  return stateInControlFrame_;
}

std::ostream& operator<<(std::ostream& out, const TorsoStateMeasured& state) {
  out << "positionWorldToBaseInWorldFrame: " << state.positionWorldToBaseInWorldFrame_ << std::endl;
  out << "orientationWorldToBase: " << state.orientationWorldToBase_ << std::endl;
  out << "orientationWorldToControl: " << state.orientationWorldToBase_ << std::endl;
  out << "linearVelocityBaseInBaseFrame: " << state.linearVelocityBaseInBaseFrame_ << std::endl;
  out << "angularVelocityBaseInBaseFrame: " << state.angularVelocityBaseInBaseFrame_ << std::endl;
  out << state.stateInControlFrame_;
  return out;
}

} /* namespace loco */
