/*
 * TorsoStateMeasuredInControlFrame.cpp
 *
 *  Created on:  May 18, 2016
 *      Author: Christian Gehring
 */

#include "loco/common/torso/TorsoStateMeasuredInControlFrame.hpp"

namespace loco {

TorsoStateMeasuredInControlFrame::TorsoStateMeasuredInControlFrame()
    : positionControlToBaseInControlFrame_(),
      orientationWorldToControl_(),
      orientationControlToBase_(),
      orientationEulerAnglesZyxControlToBase_(),
      linearVelocityBaseInControlFrame_(),
      angularVelocityBaseInControlFrame_(),
      angularVelocityControlInWorldFrame_() {}

const Position& TorsoStateMeasuredInControlFrame::getPositionControlToBaseInControlFrame() const {
  return positionControlToBaseInControlFrame_;
}

void TorsoStateMeasuredInControlFrame::setPositionControlToBaseInControlFrame(const Position& positionControlToBaseInControlFrame) {
  positionControlToBaseInControlFrame_ = positionControlToBaseInControlFrame;
}

const Position& TorsoStateMeasuredInControlFrame::getPositionWorldToControlInWorldFrame() const {
  return positionWorldToControlInWorldFrame_;
}

void TorsoStateMeasuredInControlFrame::setPositionWorldToControlInWorldFrame(const Position& positionWorldToControlInWorldFrame) {
  positionWorldToControlInWorldFrame_ = positionWorldToControlInWorldFrame;
}

void TorsoStateMeasuredInControlFrame::setOrientationWorldToControl(const RotationQuaternion& orientation) {
  orientationWorldToControl_ = orientation;
}

const RotationQuaternion& TorsoStateMeasuredInControlFrame::getOrientationWorldToControl() const {
  return orientationWorldToControl_;
}

const RotationQuaternion& TorsoStateMeasuredInControlFrame::getOrientationControlToBase() const {
  return orientationControlToBase_;
}
void TorsoStateMeasuredInControlFrame::setOrientationControlToBase(const RotationQuaternion& orientation) {
  orientationControlToBase_ = orientation;
  orientationEulerAnglesZyxControlToBase_ = EulerAnglesZyx(orientation);
  orientationEulerAnglesZyxControlToBase_.setUnique();
}

const loco::LinearVelocity& TorsoStateMeasuredInControlFrame::getLinearVelocityBaseInControlFrame() const {
  return linearVelocityBaseInControlFrame_;
}

void TorsoStateMeasuredInControlFrame::setLinearVelocityBaseInControlFrame(const LinearVelocity& linearVelocity) {
  linearVelocityBaseInControlFrame_ = linearVelocity;
}

const LocalAngularVelocity& TorsoStateMeasuredInControlFrame::getAngularVelocityBaseInControlFrame() const {
  return angularVelocityBaseInControlFrame_;
}

void TorsoStateMeasuredInControlFrame::setAngularVelocityBaseInControlFrame(const loco::LocalAngularVelocity& angularVelocity) {
  angularVelocityBaseInControlFrame_ = angularVelocity;
}

const LocalAngularVelocity& TorsoStateMeasuredInControlFrame::getAngularVelocityControlInWorldFrame() const {
  return angularVelocityControlInWorldFrame_;
}

void TorsoStateMeasuredInControlFrame::setAngularVelocityControlInWorldFrame(const loco::LocalAngularVelocity& angularVelocity) {
  angularVelocityControlInWorldFrame_ = angularVelocity;
}

const EulerAnglesZyx& TorsoStateMeasuredInControlFrame::getOrientationEulerAnglesZyxControlToBase() const {
  return orientationEulerAnglesZyxControlToBase_;
}

std::ostream& operator<<(std::ostream& out, const TorsoStateMeasuredInControlFrame& state) {
  out << "positionControlToBaseInControlFrame: " << state.positionControlToBaseInControlFrame_ << std::endl;
  out << "positionControlToBaseInControlFrame: " << state.positionControlToBaseInControlFrame_ << std::endl;
  out << "orientationControlToBase: " << state.orientationControlToBase_ << std::endl;
  out << "linearVelocityBaseInControlFrame: " << state.linearVelocityBaseInControlFrame_ << std::endl;
  out << "angularVelocityBaseInControlFrame: " << state.angularVelocityBaseInControlFrame_ << std::endl;
  out << "angularVelocityControlInWorldFrame: " << state.angularVelocityControlInWorldFrame_;
  return out;
}

} /* namespace loco */
