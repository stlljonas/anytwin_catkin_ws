/*
 * MissionControlSpeedTrajectory.cpp
 *
 *  Created on: Apr 4, 2014
 *      Author: Christian Gehring
 */

// loco
#include "loco/mission_control/MissionControlSpeedTrajectory.hpp"

namespace loco {

MissionControlSpeedTrajectory::MissionControlSpeedTrajectory()
    : MissionControlBase(), time_(0.0), currentBaseTwistInControlFrame_(), isInterpolatingTime_(true), cycleDuration_(0.0) {}

const Twist& MissionControlSpeedTrajectory::getDesiredBaseTwistInControlFrame() const {
  return currentBaseTwistInControlFrame_;
}

bool MissionControlSpeedTrajectory::initialize(double dt) {
  currentBaseTwistInControlFrame_.setZero();
  time_ = 0.0;
  return true;
}
bool MissionControlSpeedTrajectory::advance(double dt) {
  currentBaseTwistInControlFrame_.getTranslationalVelocity() = linearVelocityTrajectory_.evaluate_linear(time_);
  currentBaseTwistInControlFrame_.getRotationalVelocity() = localAngularVelocityTrajectory_.evaluate_linear(time_);
  time_ += dt;

  return true;
}
bool MissionControlSpeedTrajectory::loadParameters(const TiXmlHandle& handle) {
  linearVelocityTrajectory_.clear();
  localAngularVelocityTrajectory_.clear();

  TiXmlElement* pElem;
  double t, value;

  robot_utils::catmull_rom::TrajectoryLinearVelocity::Type linearVelocity;
  robot_utils::catmull_rom::TrajectoryLocalAngularVelocity::Type localAngularVelocity;

  TiXmlHandle hSpeedTrajectory(handle.FirstChild("Mission").FirstChild("Speed").FirstChild("Trajectory"));
  pElem = hSpeedTrajectory.ToElement();
  if (pElem == nullptr) {
    printf("Could not find LocomotionController:Mission:Speed:Trajectory!\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("cycleDuration", &cycleDuration_) == TIXML_SUCCESS) {
    isInterpolatingTime_ = false;
  } else {
    isInterpolatingTime_ = true;
  }
  TiXmlElement* child = hSpeedTrajectory.FirstChild().ToElement();
  for (; child != nullptr; child = child->NextSiblingElement()) {
    if (child->QueryDoubleAttribute("t", &t) != TIXML_SUCCESS) {
      printf("Could not find t  of knot!\n");
      return false;
    }
    if (child->QueryDoubleAttribute("headingSpeed", &linearVelocity.x()) != TIXML_SUCCESS) {
      printf("Could not find headingSpeed of knot!\n");
      return false;
    }
    if (child->QueryDoubleAttribute("lateralSpeed", &linearVelocity.y()) != TIXML_SUCCESS) {
      printf("Could not find lateralSpeed of knot!\n");
      return false;
    }
    if (child->QueryDoubleAttribute("turningSpeed", &localAngularVelocity.z()) != TIXML_SUCCESS) {
      printf("Could not find turningSpeed of knot!\n");
      return false;
    }
    if (!isInterpolatingTime_) {
      t *= cycleDuration_;
    }
    linearVelocityTrajectory_.addKnot(t, linearVelocity);
    localAngularVelocityTrajectory_.addKnot(t, localAngularVelocity);
  }
  return true;
}

const Twist& MissionControlSpeedTrajectory::getMaximumBaseTwistInControlFrame() const {}

const Pose& MissionControlSpeedTrajectory::getMinimalPoseOffset() const {}

const Pose& MissionControlSpeedTrajectory::getMaximalPoseOffset() const {}

} /* namespace loco */
