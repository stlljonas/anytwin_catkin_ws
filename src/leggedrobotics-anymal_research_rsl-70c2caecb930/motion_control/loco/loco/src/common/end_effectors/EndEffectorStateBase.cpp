/*
 * EndEffectorStateBase.cpp
 *
 *  Created on: Nov, 2017
 *      Author: Gabriel Hottiger
 */

// loco
#include <loco/common/end_effectors/EndEffectorStateBase.hpp>

namespace loco {

EndEffectorStateBase::EndEffectorStateBase()
    : positionWorldToEndEffectorInWorldFrame_(),
      linearVelocityEndEffectorInWorldFrame_(),
      linearAccelerationEndEffectorInWorldFrame_(),
      orientationWorldToEndEffector_(),
      angularVelocityEndEffectorInWorldFrame_(),
      angularAccelerationEndEffectorInWorldFrame_(),
      forceAtEndEffectorInWorldFrame_(),
      torqueAtEndEffectorInWorldFrame_() {}

void EndEffectorStateBase::setPositionWorldToEndEffectorInWorldFrame(const Position& positionWorldToEndEffectorInWorldFrame) {
  positionWorldToEndEffectorInWorldFrame_ = positionWorldToEndEffectorInWorldFrame;
}

const Position& EndEffectorStateBase::getPositionWorldToEndEffectorInWorldFrame() const {
  return positionWorldToEndEffectorInWorldFrame_;
}

void EndEffectorStateBase::setLinearVelocityEndEffectorInWorldFrame(const LinearVelocity& linearVelocityEndEffectorInWorldFrame) {
  linearVelocityEndEffectorInWorldFrame_ = linearVelocityEndEffectorInWorldFrame;
}

const LinearVelocity& EndEffectorStateBase::getLinearVelocityEndEffectorInWorldFrame() const {
  return linearVelocityEndEffectorInWorldFrame_;
}

void EndEffectorStateBase::setLinearAccelerationEndEffectorInWorldFrame(
    const LinearAcceleration& linearAccelerationEndEffectorInWorldFrame) {
  linearAccelerationEndEffectorInWorldFrame_ = linearAccelerationEndEffectorInWorldFrame;
}

const LinearAcceleration& EndEffectorStateBase::getLinearAccelerationEndEffectorInWorldFrame() const {
  return linearAccelerationEndEffectorInWorldFrame_;
}

void EndEffectorStateBase::setOrientationWorldToEndEffector(const RotationQuaternion& orientationWorldToEndEffector) {
  orientationWorldToEndEffector_ = orientationWorldToEndEffector;
}

const RotationQuaternion& EndEffectorStateBase::getOrientationWorldToEndEffector() const {
  return orientationWorldToEndEffector_;
}

void EndEffectorStateBase::setAngularVelocityEndEffectorInWorldFrame(const LocalAngularVelocity& angularVelocityEndEffectorInWorldFrame) {
  angularVelocityEndEffectorInWorldFrame_ = angularVelocityEndEffectorInWorldFrame;
}

const LocalAngularVelocity& EndEffectorStateBase::getAngularVelocityEndEffectorInWorldFrame() const {
  return angularVelocityEndEffectorInWorldFrame_;
}

void EndEffectorStateBase::setAngularAccelerationEndEffectorInWorldFrame(
    const AngularAcceleration& angularAccelerationEndEffectorInWorldFrame) {
  angularAccelerationEndEffectorInWorldFrame_ = angularAccelerationEndEffectorInWorldFrame;
}

const AngularAcceleration& EndEffectorStateBase::getAngularAccelerationEndEffectorInWorldFrame() const {
  return angularAccelerationEndEffectorInWorldFrame_;
}

const Force& EndEffectorStateBase::getForceAtEndEffectorInWorldFrame() const {
  return forceAtEndEffectorInWorldFrame_;
}

void EndEffectorStateBase::setForceAtEndEffectorInWorldFrame(const Force& forceAtEndEffectorInWorldFrame) {
  forceAtEndEffectorInWorldFrame_ = forceAtEndEffectorInWorldFrame;
}

const Torque& EndEffectorStateBase::getTorqueAtEndEffectorInWorldFrame() const {
  return torqueAtEndEffectorInWorldFrame_;
}

void EndEffectorStateBase::setTorqueAtEndEffectorInWorldFrame(const Torque& torqueAtEndEffectorInWorldFrame) {
  torqueAtEndEffectorInWorldFrame_ = torqueAtEndEffectorInWorldFrame;
}

} /* namespace loco */
