/*
 * TorsoProperties.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/common/torso/TorsoProperties.hpp"

namespace loco {

TorsoProperties::TorsoProperties()
    : mass_(0.0), inertiaTensorInBaseFrame_(), positionBaseToCenterOfMassInBaseFrame_(), maximumBaseTwistInControlFrame_() {}

double TorsoProperties::getMass() const {
  return mass_;
}

void TorsoProperties::setMass(double mass) {
  mass_ = mass;
}

const Eigen::Matrix3d& TorsoProperties::getInertiaTensorInBaseFrame() const {
  return inertiaTensorInBaseFrame_;
}

void TorsoProperties::setInertiaTensorInBaseFrame(const Eigen::Matrix3d& inertiaTensorInBaseFrame) {
  inertiaTensorInBaseFrame_ = inertiaTensorInBaseFrame;
}

const Position& TorsoProperties::getBaseToCenterOfMassPositionInBaseFrame() const {
  return positionBaseToCenterOfMassInBaseFrame_;
}

void TorsoProperties::setBaseToCenterOfMassPositionInBaseFrame(const Position& centerOfMassInBaseFrame) {
  positionBaseToCenterOfMassInBaseFrame_ = centerOfMassInBaseFrame;
}

const LinearAcceleration& TorsoProperties::getGravity() const {
  return gravity_;
}

void TorsoProperties::setGravity(const LinearAcceleration& gravity) {
  gravity_ = gravity;
  gravityAxisInWorldFrame_ = Vector(gravity).normalized();
}

const Vector& TorsoProperties::getHeadingAxisInBaseFrame() {
  return headingAxisInBaseFrame_;
}

const Vector& TorsoProperties::getLateralAxisInBaseFrame() {
  return lateralAxisInBaseFrame_;
}

const Vector& TorsoProperties::getVerticalAxisInBaseFrame() {
  return verticalAxisInBaseFrame_;
}

void TorsoProperties::setHeadingAxisInBaseFrame(const Vector& axis) {
  headingAxisInBaseFrame_ = axis;
}

void TorsoProperties::setLateralAxisInBaseFrame(const Vector& axis) {
  lateralAxisInBaseFrame_ = axis;
}

void TorsoProperties::setVerticalAxisInBaseFrame(const Vector& axis) {
  verticalAxisInBaseFrame_ = axis;
}

const Vector& TorsoProperties::getGravityAxisInWorldFrame() {
  return gravityAxisInWorldFrame_;
}

void TorsoProperties::setMaximumBaseTwistInControlFrame(const Twist& maximumBaseTwistInControlFrame) {
  maximumBaseTwistInControlFrame_ = maximumBaseTwistInControlFrame;
}

const Twist& TorsoProperties::getMaximumBaseTwistInControlFrame() const {
  return maximumBaseTwistInControlFrame_;
}

} /* namespace loco */
