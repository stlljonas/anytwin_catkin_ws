/*!
 * @file    MotionGeneration.cpp
 * @author  Alexander Reske
 * @brief   Motion generator for the controller JointConfigurationsController.
 */

// stl
#include <algorithm>
#include <limits>

// anymal_ctrl_joint_configurations
#include "anymal_ctrl_joint_configurations/MotionGeneration.hpp"

namespace anymal_ctrl_joint_configurations {

MotionGeneration::MotionGeneration()
    : time_(0.0),
      duration_(0.0),
      maxJointVelocity_(0.0),
      startJointPositions_(JointVector::Zero()),
      goalJointPositions_(JointVector::Zero()),
      desJointPositions_(JointVector::Zero()),
      desJointVelocities_(JointVector::Zero()) {}

bool MotionGeneration::create() {
  return true;
}

bool MotionGeneration::initialize() {
  time_ = 0.0;
  duration_ = 0.0;
  startJointPositions_.setZero();
  goalJointPositions_.setZero();
  desJointPositions_.setZero();
  desJointVelocities_.setZero();
  return true;
}

bool MotionGeneration::advance(double dt) {
  // linear interpolation for now
  if (duration_ <= 0.0) {
    // should never happen
    desJointPositions_ = startJointPositions_;
    desJointVelocities_.setZero();
  } else if (0.0 <= time_ && time_ < duration_) {
    desJointPositions_ = startJointPositions_ + time_ / duration_ * (goalJointPositions_ - startJointPositions_);
    desJointVelocities_ = (goalJointPositions_ - startJointPositions_) / duration_;
  } else if (time_ >= duration_) {
    desJointPositions_ = goalJointPositions_;
    desJointVelocities_.setZero();
  }
  time_ += dt;
  return true;
}

bool MotionGeneration::reset() {
  return initialize();
}

double MotionGeneration::getTime() const {
  return time_;
}

double MotionGeneration::getDuration() const {
  return duration_;
}

void MotionGeneration::setMaxJointVelocity(const double maxJointVelocity) {
  maxJointVelocity_ = maxJointVelocity;
}

void MotionGeneration::setStartJointPositions(const JointVector& startJointPositions) {
  startJointPositions_ = startJointPositions;
}

void MotionGeneration::setGoalJointPositions(const JointVector& goalJointPositions) {
  goalJointPositions_ = goalJointPositions;
}

const MotionGeneration::JointVector& MotionGeneration::getDesJointPositions() const {
  return desJointPositions_;
}

const MotionGeneration::JointVector& MotionGeneration::getDesJointVelocities() const {
  return desJointVelocities_;
}

void MotionGeneration::planMotion() {
  time_ = 0.0;
  double maxAbsJointPositionError = (goalJointPositions_ - startJointPositions_).cwiseAbs().maxCoeff();
  if (maxJointVelocity_ == 0.0) {
    duration_ = std::numeric_limits<double>::epsilon();
  } else {
    duration_ = std::max(std::numeric_limits<double>::epsilon(), maxAbsJointPositionError / maxJointVelocity_);
  }
}

} /* namespace anymal_ctrl_joint_configurations */
