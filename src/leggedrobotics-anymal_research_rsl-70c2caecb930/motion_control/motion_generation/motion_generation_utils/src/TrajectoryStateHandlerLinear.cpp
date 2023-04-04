/*
 * TrajectoryStateHandlerLinear.cpp
 *
 *  Created on: 07.09, 2018
 *      Author: Fabian Jenelten
 */

// motion generation utils
#include "motion_generation_utils/TrajectoryStateHandlerLinear.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace motion_generation {

TrajectoryStateHandlerLinear::TrajectoryStateHandlerLinear():
    TrajectoryStateHandlerBase()
{
  optimizationState_.clear();
  optimizationState_.reserve(zmp::optimizationTranslationalDofs.size());
  for(const auto& dim : zmp::optimizationTranslationalDofs) {
    optimizationState_.push_back(dim);
  }
}

Position TrajectoryStateHandlerLinear::getPositionPlaneToComInPlaneFrame() const {
  return Position(
      motionPlan_[zmp::CogDim::x].getPosition(),
      motionPlan_[zmp::CogDim::y].getPosition(),
      motionPlan_[zmp::CogDim::z].getPosition());
}

LinearVelocity TrajectoryStateHandlerLinear::getLinearVelocityComInPlaneFrame() const {
  return LinearVelocity(
      motionPlan_[zmp::CogDim::x].getVelocity(),
      motionPlan_[zmp::CogDim::y].getVelocity(),
      motionPlan_[zmp::CogDim::z].getVelocity());
}

LinearAcceleration TrajectoryStateHandlerLinear::getLinearAccelerationComInPlaneFrame() const {
  return LinearAcceleration(
      motionPlan_[zmp::CogDim::x].getAcceleration(),
      motionPlan_[zmp::CogDim::y].getAcceleration(),
      motionPlan_[zmp::CogDim::z].getAcceleration());
}

Position TrajectoryStateHandlerLinear::getPositionPlaneToComInPlaneFrameAtTime(const double tk) const {
  return Position(
      motionPlan_[zmp::CogDim::x].getPositionAtTime(tk),
      motionPlan_[zmp::CogDim::y].getPositionAtTime(tk),
      motionPlan_[zmp::CogDim::z].getPositionAtTime(tk));
}

LinearVelocity TrajectoryStateHandlerLinear::getLinearVelocityComInPlaneFrameAtTime(const double tk) const {
  return LinearVelocity(
      motionPlan_[zmp::CogDim::x].getVelocityAtTime(tk),
      motionPlan_[zmp::CogDim::y].getVelocityAtTime(tk),
      motionPlan_[zmp::CogDim::z].getVelocityAtTime(tk));
}

LinearAcceleration TrajectoryStateHandlerLinear::getLinearAccelerationComInPlaneFrameAtTime(const double tk) const {
  return LinearAcceleration(
      motionPlan_[zmp::CogDim::x].getAccelerationAtTime(tk),
      motionPlan_[zmp::CogDim::y].getAccelerationAtTime(tk),
      motionPlan_[zmp::CogDim::z].getAccelerationAtTime(tk));
}

void TrajectoryStateHandlerLinear::getPositionPlaneToComInPlaneFrame(Position& ComStateInPlaneFrame) const {
  for (const auto& dim : zmp::optimizationTranslationalDofs) {
    ComStateInPlaneFrame(zmp::toIndex(dim)) = motionPlan_[dim].getPosition();
  }
}

void TrajectoryStateHandlerLinear::getLinearVelocityComInPlaneFrame(LinearVelocity& ComStateInPlaneFrame) const {
  for (const auto& dim : zmp::optimizationTranslationalDofs) {
    ComStateInPlaneFrame(zmp::toIndex(dim)) = motionPlan_[dim].getVelocity();
  }
}

void TrajectoryStateHandlerLinear::getLinearAccelerationComInPlaneFrame(LinearAcceleration& ComStateInPlaneFrame) const {
  for (const auto& dim : zmp::optimizationTranslationalDofs) {
    ComStateInPlaneFrame(zmp::toIndex(dim)) = motionPlan_[dim].getAcceleration();
  }
}

void TrajectoryStateHandlerLinear::getPositionPlaneToComInPlaneFrameAtTime(Position& ComStateInPlaneFrame, double tk) const {
  for (const auto& dim : zmp::optimizationTranslationalDofs) {
    ComStateInPlaneFrame(zmp::toIndex(dim)) = motionPlan_[dim].getPositionAtTime(tk);
  }
}

void TrajectoryStateHandlerLinear::getLinearVelocityComInPlaneFrameAtTime(LinearVelocity& ComStateInPlaneFrame, double tk) const {
  for (const auto& dim : zmp::optimizationTranslationalDofs) {
    ComStateInPlaneFrame(zmp::toIndex(dim)) = motionPlan_[dim].getVelocityAtTime(tk);
  }
}

void TrajectoryStateHandlerLinear::getLinearAccelerationComInPlaneFrameAtTime(LinearAcceleration& ComStateInPlaneFrame, double tk) const {
  for (const auto& dim : zmp::optimizationTranslationalDofs) {
    ComStateInPlaneFrame(zmp::toIndex(dim)) = motionPlan_[dim].getAccelerationAtTime(tk);
  }
}

bool TrajectoryStateHandlerLinear::setMotionPlanLinear(
    const positionVector& positionKnotInPlaneFrame,
    const velocityVector& velocityKnotInPlaneFrame,
    const accelerationVector& accelerationKnotInPlaneFrame,
    const std::vector<double>& duration,
    double optimizationHorizonInSeconds) {
  // Clear all splines.
  if(!reset()) { return false; }

  // Add splines for translational part.
  if(!addMotionPlanTranslational(
      positionKnotInPlaneFrame,
      velocityKnotInPlaneFrame,
      accelerationKnotInPlaneFrame,
      duration,
      optimizationHorizonInSeconds)) { return false; }

  // Initialize container.
  setContainerDuration(optimizationHorizonInSeconds);
  if(!setContainerTime(0.0)) { return false; }
  return true;
}

bool TrajectoryStateHandlerLinear::addMotionPlanTranslational(
    const positionVector& positionKnotInPlaneFrame,
    const velocityVector& velocityKnotInPlaneFrame,
    const accelerationVector& accelerationKnotInPlaneFrame,
    const std::vector<double>& duration,
    double optimizationHorizonInSeconds) {

  if (duration.size()+1u != positionKnotInPlaneFrame.size() || positionKnotInPlaneFrame.size() != velocityKnotInPlaneFrame.size()) {
    MELO_WARN_STREAM("[TrajectoryStateHandlerBase::setMotionPlan] Wrong vector dimension!");
    return false;
  }

  curves::SplineOptions opts;
  for (auto splineId=0u; splineId<duration.size(); ++splineId) {
    for (const auto& dim : zmp::optimizationTranslationalDofs) {
      opts.tf_    = duration[splineId];
      opts.pos0_  = positionKnotInPlaneFrame[splineId](zmp::toIndex(dim));
      opts.posT_  = positionKnotInPlaneFrame[splineId+1](zmp::toIndex(dim));
      opts.vel0_  = velocityKnotInPlaneFrame[splineId](zmp::toIndex(dim));
      opts.velT_  = velocityKnotInPlaneFrame[splineId+1](zmp::toIndex(dim));
      opts.acc0_  = accelerationKnotInPlaneFrame[splineId](zmp::toIndex(dim));
      opts.accT_  = accelerationKnotInPlaneFrame[splineId+1](zmp::toIndex(dim));
      if(!addSpline(curves::PolynomialSplineQuintic(opts), dim)) { return false; }
    }
  }
  return true;
}

} /* namespace motion_generation */
