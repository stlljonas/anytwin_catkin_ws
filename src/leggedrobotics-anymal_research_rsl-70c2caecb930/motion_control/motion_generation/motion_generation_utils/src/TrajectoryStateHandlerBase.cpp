/*
 * TrajectoryStateHandlerBase.cpp
 *
 *  Created on: 05.08, 2017
 *      Author: Fabian Jenelten
 */

// motion generation utils
#include "motion_generation_utils/TrajectoryStateHandlerBase.hpp"

// message logger
#include "message_logger/message_logger.hpp"


namespace motion_generation {

TrajectoryStateHandlerBase::TrajectoryStateHandlerBase():
  motionPlan_(curves::PolynomialSplineContainerQuintic()),
  containerTime_(0.0),
  containerDuration_(0.0),
  optimizationState_()
{

}

bool TrajectoryStateHandlerBase::initialize() {
  containerDuration_  = 0.0;
  containerTime_      = 0.0;
  if(!reset()) { return false; }
  return true;
}

bool TrajectoryStateHandlerBase::isEmpty(bool verbose) const {
  for (const auto& dim : optimizationState_) {
    if (motionPlan_[dim].isEmpty()) {
      if (verbose) {
        MELO_WARN_STREAM("[TrajectoryStateHandlerBase::isEmpty] Motion plan is empty at dim " << zmp::cogDimMap[dim] << ".");
      }
      return true;
    }
  }
  return false;
}


double TrajectoryStateHandlerBase::getComStateInPlaneFrameAtTime(
      double tk,
      zmp::CogDim dim,
      zmp::Derivative derivative) const {
  if(derivative==zmp::Derivative::Zero) {
    return motionPlan_[dim].getPositionAtTime(tk);
  } else if(derivative==zmp::Derivative::First) {
    return motionPlan_[dim].getVelocityAtTime(tk);
  } else if(derivative==zmp::Derivative::Second) {
    return motionPlan_[dim].getAccelerationAtTime(tk);
  }

  MELO_FATAL_STREAM("[TrajectoryStateHandlerBase::getComStateInPlaneFrameAtTime] Undefined derivative!");
  return 0.0;
}

double TrajectoryStateHandlerBase::getFinalComStateInPlaneFrame(
      zmp::CogDim dim,
      zmp::Derivative derivative) const {
  if(derivative==zmp::Derivative::Zero) {
    return motionPlan_[dim].getEndPosition();
  } else if(derivative==zmp::Derivative::First) {
    return motionPlan_[dim].getEndVelocity();
  } else if(derivative==zmp::Derivative::Second) {
    return motionPlan_[dim].getEndAcceleration();
  }

  MELO_FATAL_STREAM("[TrajectoryStateHandlerBase::getFinalComStateInPlaneFrame] Undefined derivative!");
  return 0.0;
}

bool TrajectoryStateHandlerBase::advance(double dt, bool verbose) {
  for (const auto& dim : optimizationState_) {
    if (!motionPlan_[dim].advance(dt)) {
      if (verbose) {
        MELO_WARN_STREAM("[MotionPlan::advance] Failed to advance motion plan at dim = " << zmp::cogDimMap[dim] <<
            ". Actual container time = " << containerTime_ << ", total container duration = " << containerDuration_ <<
            ". Actual time at dim = " << motionPlan_[dim].getContainerTime() <<
            ". Container duration at dim = " << motionPlan_[dim].getContainerDuration());
      }
      return false;
    }
  }
  containerTime_ += dt;
  return true;
}

double TrajectoryStateHandlerBase::getContainerDuration() const {
  return containerDuration_;
}

void TrajectoryStateHandlerBase::updateContainerDuration() {
  containerDuration_ = motionPlan_.front().getContainerDuration();
  for (const auto& dim : optimizationState_) {
    containerDuration_ = std::fmin(motionPlan_[dim].getContainerDuration(), containerDuration_);
  }
}

void TrajectoryStateHandlerBase::setContainerDuration(double containerDuration) {
  containerDuration_ = containerDuration;
}

double TrajectoryStateHandlerBase::getContainerTime() const {
  return containerTime_;
}

const TrajectoryStateHandlerBase::MotionPlan& TrajectoryStateHandlerBase::getMotionPlan() const {
  return motionPlan_;
}

void TrajectoryStateHandlerBase::setMotionPlan(const MotionPlan& motionPlan) {
  motionPlan_ = motionPlan;
}

void TrajectoryStateHandlerBase::setMotionPlan(const curves::PolynomialSplineContainerQuintic& splineContainer, zmp::CogDim dim) {
  motionPlan_[dim] = splineContainer;
}

bool TrajectoryStateHandlerBase::setMotionPlan(const curves::PolynomialSplineQuintic& spline, zmp::CogDim dim) {
  return ( motionPlan_[dim].reset() && motionPlan_[dim].addSpline(spline) );
}

bool TrajectoryStateHandlerBase::setContainerTime(double containerTime) {
  if (isEmpty(true)) { return false; }
  if (containerTime>containerDuration_) {
    MELO_WARN_STREAM("[TrajectoryStateHandlerBase::setContainerTime] Container time " << containerTime <<
        " is larger than container duration " << containerDuration_ << ". Clip container time to constraints.");
    containerTime = containerDuration_;
  }

  for (const auto& dim : optimizationState_) {
    motionPlan_[dim].resetTime();
    motionPlan_[dim].setContainerTime(containerTime);
  }
  containerTime_ = containerTime;
  return true;
}

bool TrajectoryStateHandlerBase::reset(zmp::CogDim dim) {
  return motionPlan_[dim].reset();
}

bool TrajectoryStateHandlerBase::reset() {
  for (auto& motionPlan : motionPlan_) {
    if(!motionPlan.reset()) { return false; }
  }
  return true;
}

bool TrajectoryStateHandlerBase::addSpline(const curves::PolynomialSplineQuintic& spline, zmp::CogDim dim) {
  return motionPlan_[dim].addSpline(spline);
}

bool TrajectoryStateHandlerBase::addSpline(curves::PolynomialSplineQuintic&& spline, zmp::CogDim dim) {
  return motionPlan_[dim].addSpline(std::move(spline));
}

bool TrajectoryStateHandlerBase::checkTrajectoryStateHandler() const {
  for (const auto& dim : optimizationState_) {
    if(!motionPlan_[dim].checkContainer()) { return false; }
  }

  if (containerDuration_<0.0) { return false; }
  if (containerTime_<0.0) { return false; }
  if (containerTime_>containerDuration_) { return false; }

  return true;
}


} /* namespace motion_generation */
