/*
 * MeasuredJointStates.cpp
 *
 *  Created on: Feb 13, 2017
 *      Author: Gabriel Hottiger
 */

#include "loco/common/joints/MeasuredJointStates.hpp"

// STL
#include <cassert>

namespace loco {

MeasuredJointStates::MeasuredJointStates(const unsigned int nJoints) : JointStates(nJoints) {
  gravityJointTorques_.resize(nJoints_, 1);
  gravityJointTorques_.setZero();
  jointMinPositions_.resize(nJoints_, 1);
  jointMinPositions_.setZero();
  jointMaxPositions_.resize(nJoints_, 1);
  jointMaxPositions_.setZero();
  jointMinVelocities_.resize(nJoints_, 1);
  jointMinVelocities_.setZero();
  jointMaxVelocities_.resize(nJoints_, 1);
  jointMaxVelocities_.setZero();
}

const JointTorques& MeasuredJointStates::getGravityJointTorques() const {
  return gravityJointTorques_;
}

void MeasuredJointStates::setGravityJointTorques(const JointTorques& gravityJointTorques) {
  assert(gravityJointTorques.rows() == gravityJointTorques_.rows());
  gravityJointTorques_ = gravityJointTorques;
}

void MeasuredJointStates::setGravityJointTorque(const unsigned int index, const JointTorque& gravityJointTorque) {
  assert(index < gravityJointTorques_.rows());
  gravityJointTorques_(index) = gravityJointTorque;
}

const JointPositions& MeasuredJointStates::getJointMinPositions() const {
  return jointMinPositions_;
}

void MeasuredJointStates::setJointMinPosition(const unsigned int index, const JointPosition& jointMinPosition) {
  assert(index < jointMinPositions_.rows());
  jointMinPositions_(index) = jointMinPosition;
}

const JointPositions& MeasuredJointStates::getJointMaxPositions() const {
  return jointMaxPositions_;
}

void MeasuredJointStates::setJointMaxPosition(const unsigned int index, const JointPosition& jointMaxPosition) {
  assert(index < jointMaxPositions_.rows());
  jointMaxPositions_(index) = jointMaxPosition;
}

const JointVelocities& MeasuredJointStates::getJointMinVelocities() const {
  return jointMinVelocities_;
}

void MeasuredJointStates::setJointMinVelocity(const unsigned int index, const JointVelocity& jointMinVelocity) {
  assert(index < jointMinVelocities_.rows());
  jointMinVelocities_(index) = jointMinVelocity;
}

const JointVelocities& MeasuredJointStates::getJointMaxVelocities() const {
  return jointMaxVelocities_;
}

void MeasuredJointStates::setJointMaxVelocity(const unsigned int index, const JointVelocity& jointMaxVelocity) {
  assert(index < jointMaxVelocities_.rows());
  jointMaxVelocities_(index) = jointMaxVelocity;
}

} /* namespace loco */
