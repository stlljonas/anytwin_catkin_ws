/*
 * MotionPlan.cpp
 *
 *  Created on: Jan. 15, 2017
 *      Author: Fabian Jenelten
 */

// swing trajectory generation
#include "swing_trajectory_generation/MotionPlan.hpp"


namespace sto {

MotionPlan::MotionPlan():
    MotionPlanLinear(),
    swingTrajectoryInPlaneFrame_(),
    numOfTotSwingSplines_(0u),
    numOfTotStanceSplines_(0u),
    swingSplineDurations_(),
    swingDuration_(0.0),
    stanceDuration_(-1.0),
    timeSpendInSwing_(-1.0),
    trajectoryStartTime_(-1.0),
    isPreviousOptimizationAvailable_(true)
{

}

bool MotionPlan::initialize() {
  if(!MotionPlanLinear::initialize()) { return false; }
  optimizationDofs_ = {zmp::CogDim::x, zmp::CogDim::y, zmp::CogDim::z};
  if(!swingTrajectoryInPlaneFrame_.initialize()) { return false; }

  numOfTotStanceSplines_ = 0u;
  zmp::LineSearchOptions lineSearchOptions(1e-4, 12u, 10u, 0.4, 0.6, true, 2.0); // ToDO: weight!
  timeSpendInSwing_ = -1.0;
  return true;
}

bool MotionPlan::resetSwingTrajectory() {
  return swingTrajectoryInPlaneFrame_.initialize();
}

bool MotionPlan::setVirtualPlaneFrame(
    const motion_generation::Position& positionWorldToPreviousStanceFootholdInWorldFrame,
    const motion_generation::RotationQuaternion& orientationWorldToControl) {
  VirtualPlaneFrameFoot virtualPlaneFrameFoot;
  // Store previous virtual plane frame.
  posePreviousPlaneToWorld_ = virtualPlaneFrame_.getPosePlaneToWorld();

  // Compute a new virtual plane frame.
  if(!virtualPlaneFrameFoot.computeVirtualPlaneFrame(positionWorldToPreviousStanceFootholdInWorldFrame, orientationWorldToControl)) {
    return false;
  }
  virtualPlaneFrame_.copy(virtualPlaneFrameFoot);

  return true;
}


void MotionPlan::setTimingInformations(
    double swingDuration,
    double stanceDuration,
    double timeSpendInSwing,
    double trajectoryStartTime,
    bool isPreviousOptimizationAvailable) {
  assert(swingDuration>=0.0 && stanceDuration>=0.0);
  swingDuration_                   = swingDuration;
  stanceDuration_                  = stanceDuration;
  timeSpendInSwing_                = timeSpendInSwing;
  trajectoryStartTime_             = trajectoryStartTime;
  isPreviousOptimizationAvailable_ = isPreviousOptimizationAvailable;
  optimizationHorizon_             = swingDuration - trajectoryStartTime;
}

motion_generation::TrajectoryStateHandlerLineSearchLinear* MotionPlan::getSwingTrajectoryInPlaneFramePtr() {
  return &swingTrajectoryInPlaneFrame_;
}

const motion_generation::TrajectoryStateHandlerLineSearchLinear& MotionPlan::getSwingTrajectoryInPlaneFrame() const {
  return swingTrajectoryInPlaneFrame_;
}

const std::vector<double>& MotionPlan::getSwingSplineDurations() const {
  return swingSplineDurations_;
}

void MotionPlan::setSplineDurations(std::vector<double>& swingSplineDurations, unsigned int numOfTotSwingSplines) {
  swingSplineDurations_ = swingSplineDurations;
  numOfTotSwingSplines_ = numOfTotSwingSplines;
}

unsigned int MotionPlan::getActiveSplineId(double time) const {
  double accumulatedTime          = trajectoryStartTime_;
  const double currentTime        = time + trajectoryStartTime_;
  const unsigned int offsetId     = getSplineIdOffset();
  const unsigned int numOfSplines = swingSplineDurations_.size() + numOfTotStanceSplines_;

  for (unsigned int splineId=0u; splineId<numOfSplines; ++splineId) {
    accumulatedTime += swingSplineDurations_[splineId];
    if (accumulatedTime>currentTime) {
      return splineId+offsetId;
    }
  }
  return swingSplineDurations_.size()+offsetId;
}

unsigned int MotionPlan::getSplineIdOffset() const {
  assert(numOfTotSwingSplines_>=swingSplineDurations_.size());
  return (numOfTotSwingSplines_ - swingSplineDurations_.size());
}

double MotionPlan::getSwingDuration() const {
  return swingDuration_;
}

double MotionPlan::getSanceDuration() const {
  return stanceDuration_;
}

double MotionPlan::getTimeSpendInSwing() const {
  return timeSpendInSwing_;
}

double MotionPlan::getTrajectoryStartTime() const {
  return trajectoryStartTime_;
}

void MotionPlan::getPositionWorldToDesiredFootInWorldFrame(motion_generation::Position& positionWorldToDesiredFootInWorldFrame) const {
  swingTrajectoryInPlaneFrame_.getPositionPlaneToComInPlaneFrame(positionWorldToDesiredFootInWorldFrame);
  positionWorldToDesiredFootInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().transform(positionWorldToDesiredFootInWorldFrame);
}

void MotionPlan::getLinearVelocityDesiredFootInWorldFrame(motion_generation::LinearVelocity& linearVelocityDesiredFootInWorldFrame) const {
  swingTrajectoryInPlaneFrame_.getLinearVelocityComInPlaneFrame(linearVelocityDesiredFootInWorldFrame);
  linearVelocityDesiredFootInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(linearVelocityDesiredFootInWorldFrame);
}

void MotionPlan::getLinearAccelerationDesiredFootInWorldFrame(motion_generation::LinearAcceleration& linearAccelerationDesiredFootInWorldFrame) const {
  swingTrajectoryInPlaneFrame_.getLinearAccelerationComInPlaneFrame(linearAccelerationDesiredFootInWorldFrame);
  linearAccelerationDesiredFootInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(linearAccelerationDesiredFootInWorldFrame);
}

void MotionPlan::getPositionWorldToDesiredFootInWorldFrameAtTime(motion_generation::Position& positionWorldToDesiredFootInWorldFrame, double dt) const {
  swingTrajectoryInPlaneFrame_.getPositionPlaneToComInPlaneFrameAtTime(positionWorldToDesiredFootInWorldFrame, dt);
  positionWorldToDesiredFootInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().transform(positionWorldToDesiredFootInWorldFrame);
}

void MotionPlan::getLinearVelocityDesiredFootInWorldFrameAtTime(motion_generation::LinearVelocity& linearVelocityDesiredFootInWorldFrame, double dt) const {
  swingTrajectoryInPlaneFrame_.getLinearVelocityComInPlaneFrameAtTime(linearVelocityDesiredFootInWorldFrame, dt);
  linearVelocityDesiredFootInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(linearVelocityDesiredFootInWorldFrame);
}

void MotionPlan::getLinearAccelerationDesiredFootInWorldFrameAtTime(motion_generation::LinearAcceleration& linearAccelerationDesiredFootInWorldFrame, double dt) const {
  swingTrajectoryInPlaneFrame_.getLinearAccelerationComInPlaneFrameAtTime(linearAccelerationDesiredFootInWorldFrame, dt);
  linearAccelerationDesiredFootInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(linearAccelerationDesiredFootInWorldFrame);
}


double MotionPlan::getContainerTime() const {
  return swingTrajectoryInPlaneFrame_.getContainerTime();
}

double MotionPlan::getContainerDuration() const {
  return swingTrajectoryInPlaneFrame_.getContainerDuration();
}

bool MotionPlan::updateContainerDuration() {
  assert(optimizationHorizon_>=0.0);
  swingTrajectoryInPlaneFrame_.setContainerDuration(optimizationHorizon_);
  return true;
}

bool MotionPlan::isPreviousOptimizationAvailable() {
  return isPreviousOptimizationAvailable_;
}

void MotionPlan::copy(const MotionPlan& motionPlan) {
  terminationState_            = zmp::TerminationState::Undefined;
  numOfTotStanceSplines_       = 0u;
  swingTrajectoryInPlaneFrame_ = motionPlan.getSwingTrajectoryInPlaneFrame();
  trajectoryStartTime_         = motionPlan.getTrajectoryStartTime();
  virtualPlaneFrame_           = motionPlan.getVirtualPlaneFrame();
  timeSpendInSwing_            = motionPlan.getTimeSpendInSwing();
}

bool MotionPlan::addStancePhase() {
  if (!didOptimizationSucceeded()) { return false; }

  curves::SplineOptions opts;
  motion_generation::Position positionPlaneToFinalPositionInPlaneFrame = getPlaneToFinalPositionInPlaneFrame();
  opts.tf_ = stanceDuration_;

  //Add stance phase (in case of late touch-down we need some trajectory).
  for (const auto dim : zmp::optimizationTranslationalDofs) {
    opts.pos0_ = positionPlaneToFinalPositionInPlaneFrame(zmp::toIndex(dim));
    opts.posT_ = opts.pos0_;
    if(!swingTrajectoryInPlaneFrame_.addSpline(curves::PolynomialSplineQuintic(opts), dim)) { return false; }
  }

  const double newContainerDuration = swingTrajectoryInPlaneFrame_.getContainerDuration() + opts.tf_;
  swingTrajectoryInPlaneFrame_.setContainerDuration(newContainerDuration);
  numOfTotStanceSplines_ = 1u;
  return true;
}

} /* namespace sto */
