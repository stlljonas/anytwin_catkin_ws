/*
 * TorsoStateDesired.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

// loco
#include "loco/common/torso/TorsoStateDesired.hpp"

namespace loco {

TorsoStateDesired::TorsoStateDesired()
    : targetPoint_(TargetPoint::BASE),
      positionControlToTargetInControlFrame_(),
      orientationControlToBase_(),
      linearVelocityTargetInControlFrame_(),
      angularVelocityBaseInControlFrame_(),
      linearVelocityCommandedTargetInControlFrame_(),
      angularVelocityCommandedBaseInControlFrame_(),
      linearAccelerationTargetInControlFrame_(),
      positionErrorInControlFrame_(),
      linearVelocityErrorInControlFrame_(),
      angularVelocityErrorInControlFrame_(),
      desiredPositionOffsetInWorldFrame_(),
      desiredOrientationOffset_() {}

const Position& TorsoStateDesired::getPositionControlToTargetInControlFrame() const {
  return positionControlToTargetInControlFrame_;
}

void TorsoStateDesired::setPositionControlToTargetInControlFrame(const Position& positionControlToBaseInControlFrame) {
  positionControlToTargetInControlFrame_ = positionControlToBaseInControlFrame;
}

const Position& TorsoStateDesired::getPositionWorldToBaseInWorldFrame() const {
  return positionWorldToBaseInWorldFrame_;
}

void TorsoStateDesired::setPositionWorldToBaseInWorldFrame(const Position& positionWorldToBaseInWorldFrame) {
  positionWorldToBaseInWorldFrame_ = positionWorldToBaseInWorldFrame;
}

const RotationQuaternion& TorsoStateDesired::getOrientationControlToBase() const {
  return orientationControlToBase_;
}

void TorsoStateDesired::setOrientationControlToBase(const RotationQuaternion& orientation) {
  orientationControlToBase_ = orientation;
  orientationEulerAnglesZyxControlToBase_ = EulerAnglesZyx(orientation);
  orientationEulerAnglesZyxControlToBase_.setUnique();
}

void TorsoStateDesired::setOrientationEulerAnglesZyxBaseToWorld(const EulerAnglesZyx& orientationBaseToWorld) {
  orientationEulerAnglesZyxBaseToWorld_ = orientationBaseToWorld;
}

const EulerAnglesZyx& TorsoStateDesired::getOrientationEulerAnglesZyxBaseToWorld() const {
  return orientationEulerAnglesZyxBaseToWorld_;
}

void TorsoStateDesired::setLinearVelocityTargetInControlFrame(const LinearVelocity& linearVelocity) {
  linearVelocityTargetInControlFrame_ = linearVelocity;
}

const LinearVelocity& TorsoStateDesired::getLinearVelocityTargetInControlFrame() const {
  return linearVelocityTargetInControlFrame_;
}

void TorsoStateDesired::setAngularVelocityBaseInControlFrame(const LocalAngularVelocity& angularVelocity) {
  angularVelocityBaseInControlFrame_ = angularVelocity;
}

const LocalAngularVelocity& TorsoStateDesired::getAngularVelocityBaseInControlFrame() const {
  return angularVelocityBaseInControlFrame_;
}

void TorsoStateDesired::setLinearVelocityCommandedTargetInControlFrame(const LinearVelocity& linearVelocity) {
  linearVelocityCommandedTargetInControlFrame_ = linearVelocity;
}

const LinearVelocity& TorsoStateDesired::getLinearVelocityCommandedTargetInControlFrame() const {
  return linearVelocityCommandedTargetInControlFrame_;
}

void TorsoStateDesired::setAngularVelocityCommandedBaseInControlFrame(const LocalAngularVelocity& angularVelocity) {
  angularVelocityCommandedBaseInControlFrame_ = angularVelocity;
}

const LocalAngularVelocity& TorsoStateDesired::getAngularVelocityCommandedBaseInControlFrame() const {
  return angularVelocityCommandedBaseInControlFrame_;
}

void TorsoStateDesired::setLinearAccelerationTargetInControlFrame(const LinearAcceleration& linearAcceleration) {
  linearAccelerationTargetInControlFrame_ = linearAcceleration;
}

const LinearAcceleration& TorsoStateDesired::getLinearAccelerationTargetInControlFrame() const {
  return linearAccelerationTargetInControlFrame_;
}

void TorsoStateDesired::setAngularAccelerationTargetInControlFrame(const AngularAcceleration& angularAcceleration) {
  angularAccelerationTargetInControlFrame_ = angularAcceleration;
}

const AngularAcceleration& TorsoStateDesired::getAngularAccelerationTargetInControlFrame() const {
  return angularAccelerationTargetInControlFrame_;
}

const Position& TorsoStateDesired::getPositionErrorInControlFrame() const {
  return positionErrorInControlFrame_;
}

void TorsoStateDesired::setPositionErrorInControlFrame(const Position& positionErrorInControlFrame) {
  positionErrorInControlFrame_ = positionErrorInControlFrame;
}

const LinearVelocity& TorsoStateDesired::getLinearVelocityErrorInControlFrame() const {
  return linearVelocityErrorInControlFrame_;
}

void TorsoStateDesired::setLinearVelocityErrorInControlFrame(const LinearVelocity& linearVelocityErrorInControlFrame) {
  linearVelocityErrorInControlFrame_ = linearVelocityErrorInControlFrame;
}

const LocalAngularVelocity& TorsoStateDesired::getAngularVelocityErrorInControlFrame() const {
  return angularVelocityErrorInControlFrame_;
}

void TorsoStateDesired::setAngularVelocityErrorInControlFrame(const LocalAngularVelocity& angularVelocityErrorInControlFrame) {
  angularVelocityErrorInControlFrame_ = angularVelocityErrorInControlFrame;
}

const EulerAnglesZyx& TorsoStateDesired::getOrientationEulerAnglesZyxControlToBase() const {
  return orientationEulerAnglesZyxControlToBase_;
}

void TorsoStateDesired::setDesiredPositionOffsetInWorldFrame(const Position& positionTargetOffsetInWorldFrame) {
  desiredPositionOffsetInWorldFrame_ = positionTargetOffsetInWorldFrame;
}

void TorsoStateDesired::setDesiredOrientationOffset(const RotationQuaternion& orientationOffset) {
  desiredOrientationOffset_ = orientationOffset;
}

const Position& TorsoStateDesired::getDesiredPositionOffsetInWorldFrame() const {
  return desiredPositionOffsetInWorldFrame_;
}

const RotationQuaternion& TorsoStateDesired::getDesiredOrientationOffset() const {
  return desiredOrientationOffset_;
}

std::ostream& operator<<(std::ostream& out, const TorsoStateDesired& desiredState) {
  out << "positionControlToBaseInControlFrame: " << desiredState.positionControlToTargetInControlFrame_ << std::endl;
  out << "orientationControlToBase: " << desiredState.orientationControlToBase_ << std::endl;
  out << "linearVelocityBaseInControlFrame: " << desiredState.linearVelocityTargetInControlFrame_ << std::endl;
  out << "angularVelocityBaseInControlFrame: " << desiredState.angularVelocityBaseInControlFrame_ << std::endl;
  out << "linearAccelerationBaseInControlFrame: " << desiredState.linearAccelerationTargetInControlFrame_ << std::endl;
  out << "angularAccelerationBaseInControlFrame: " << desiredState.angularAccelerationTargetInControlFrame_ << std::endl;
  out << "positionErrorInControlFrame " << desiredState.positionErrorInControlFrame_ << std::endl;
  out << "linearVelocityErrorInControlFrame " << desiredState.linearVelocityErrorInControlFrame_ << std::endl;

  return out;
}

} /* namespace loco */
