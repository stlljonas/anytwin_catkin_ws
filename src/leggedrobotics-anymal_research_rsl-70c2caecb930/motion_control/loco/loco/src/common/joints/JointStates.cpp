/*
 * JointStates.cpp
 *
 *  Created on: Feb 13, 2017
 *      Author: Gabriel Hottiger
 */

#include "loco/common/joints/JointStates.hpp"

// STL
#include <cassert>

namespace loco {

JointStates::JointStates(const unsigned int nJoints) : nJoints_(nJoints) {
  jointControlModes_.resize(nJoints_, 1);
  jointControlModes_.setZero();
  jointPositions_.resize(nJoints_, 1);
  jointPositions_.setZero();
  jointVelocities_.resize(nJoints_, 1);
  jointVelocities_.setZero();
  jointAccelerations_.resize(nJoints_, 1);
  jointAccelerations_.setZero();
  jointTorques_.resize(nJoints_, 1);
  jointTorques_.setZero();
}

const JointControlModes& JointStates::getJointControlModes() const {
  return jointControlModes_;
}

void JointStates::setJointControlModes(const JointControlModes& jointControlModes) {
  assert(jointControlModes.rows() == jointControlModes_.rows());
  jointControlModes_ = jointControlModes;
}

void JointStates::setJointControlMode(const unsigned int index, const JointControlMode& jointControlMode) {
  assert(index < jointControlModes_.rows());
  jointControlModes_(index) = jointControlMode;
}

const JointPositions& JointStates::getJointPositions() const {
  return jointPositions_;
}

void JointStates::setJointPositions(const JointPositions& jointPositions) {
  assert(jointPositions.rows() == jointPositions_.rows());
  jointPositions_ = jointPositions;
}

void JointStates::setJointPosition(const unsigned int index, const JointPosition& jointPosition) {
  assert(index < jointPositions_.rows());
  jointPositions_(index) = jointPosition;
}

const JointVelocities& JointStates::getJointVelocities() const {
  return jointVelocities_;
}

void JointStates::setJointVelocities(const JointVelocities& jointVelocities) {
  assert(jointVelocities.rows() == jointVelocities_.rows());
  jointVelocities_ = jointVelocities;
}

void JointStates::setJointVelocity(const unsigned int index, const JointVelocity& jointVelocity) {
  assert(index < jointVelocities_.rows());
  jointVelocities_(index) = jointVelocity;
}

const JointAccelerations& JointStates::getJointAccelerations() const {
  return jointAccelerations_;
}

void JointStates::setJointAccelerations(const JointAccelerations& jointAccelerations) {
  assert(jointAccelerations.rows() == jointAccelerations_.rows());
  jointAccelerations_ = jointAccelerations;
}

void JointStates::setJointAcceleration(const unsigned int index, const JointAcceleration& jointAcceleration) {
  assert(index < jointAccelerations_.rows());
  jointAccelerations_(index) = jointAcceleration;
}

const JointTorques& JointStates::getJointTorques() const {
  return jointTorques_;
}

void JointStates::setJointTorques(const JointTorques& jointTorques) {
  assert(jointTorques.rows() == jointTorques_.rows());
  jointTorques_ = jointTorques;
}

void JointStates::setJointTorque(const unsigned int index, const JointTorque& jointTorque) {
  assert(index < jointTorques_.rows());
  jointTorques_(index) = jointTorque;
}

} /* namespace loco */
