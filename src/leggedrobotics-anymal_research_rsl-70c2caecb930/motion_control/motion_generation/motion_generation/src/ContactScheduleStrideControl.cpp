/*
 * ContactScheduleStrideControl.cpp
 *
 *  Created on: June 07, 2018
 *      Author: Fabian Jenelten
 */

// loco.
#include "motion_generation/ContactScheduleStrideControl.hpp"

// signal logger.
#include "signal_logger/signal_logger.hpp"

namespace loco {

ContactScheduleStrideControl::ContactScheduleStrideControl(WholeBody& wholeBody)
    : Base(wholeBody),
      legs_(*wholeBody.getLegsPtr()),
      lumpedUnFilteredVelocityError_(0.0),
      velocityFeedbackFilter_()
{
}

bool ContactScheduleStrideControl::initialize(double dt)  {
  if (!Base::initialize(dt)) { return false; }
  velocityFeedbackFilter_.setFilterParameters(dt, 0.25, 1.0, 0.0);
  lumpedUnFilteredVelocityError_ = 0.0;
  return true;
}

bool ContactScheduleStrideControl::addVariablesToLog(const std::string & ns) const {
  signal_logger::add(lumpedUnFilteredVelocityError_, "lumpedVelocityError/unfiltered", ns);
  signal_logger::add(timeElapsedSinceGaitSwitchStart_, "timeElapsedSinceGaitSwitchStart", ns);
  signal_logger::add(velocityFeedbackFilter_.getFilteredValue(), "lumpedVelocityError/filtered", ns);
  return ContactSchedulePeriodicSwitch::addVariablesToLog(ns);
}

bool ContactScheduleStrideControl::updateVelocityFeedback(const LinearVelocity& linearVelocityDesiredInControlFrame) {
  // Merge linear limb velocity errors w.r.t. base velocity.
  Eigen::Vector3d absLinearVelocityErrorInControlFrame = Eigen::Vector3d::Zero();
  const auto& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  for (const auto leg: legs_) {
    const auto& linearVelocityMeasuredInControlFrame = orientationWorldToControl.rotate(leg->getLimbStateMeasured().getLinearVelocityLimbBaseInWorldFrame());
    absLinearVelocityErrorInControlFrame += (linearVelocityMeasuredInControlFrame-linearVelocityDesiredInControlFrame).toImplementation().cwiseAbs();
  }
  if (legs_.size() > 0u) {
    absLinearVelocityErrorInControlFrame /= static_cast<double>(legs_.size());
  } else {
    return false;
  }
  lumpedUnFilteredVelocityError_ = absLinearVelocityErrorInControlFrame.norm();

  // Smooth (with second order filter by using two first order filter in sequence).
  double lumpedFilteredVelocityError = velocityFeedbackFilter_.advance(lumpedUnFilteredVelocityError_);
  lumpedFilteredVelocityError = velocityFeedbackFilter_.advance(lumpedFilteredVelocityError);

  // Set stride feedback signal.
  getActiveGaitDescription().setStrideFeedback(-lumpedFilteredVelocityError);
  getDesiredGaitDescription().setStrideFeedback(-lumpedFilteredVelocityError);

  return true;
}

double ContactScheduleStrideControl::getLumpedFilteredVelocityError() const {
  return velocityFeedbackFilter_.getFilteredValue();
}

double ContactScheduleStrideControl::getLumpedUnFilteredVelocityError() const noexcept {
  return lumpedUnFilteredVelocityError_;
}

double ContactScheduleStrideControl::getInterpolatedStrideDuration() const {
  if (status_ == contact_schedule::Status::SwitchGait && timeElapsedSinceGaitSwitchStart_ < activeGaitDuration_) {
    return robot_utils::linearlyInterpolate(
        getActiveGaitDescription().getStrideDuration(),
        getDesiredGaitDescription().getStrideDuration(),
        0.0,
        activeGaitDuration_,
        timeElapsedSinceGaitSwitchStart_
    );
  }
  return getActiveGaitDescription().getStrideDuration();
}

}  // end namespace loco
