/*
 * TrajectoryStateHandlerAngular.cpp
 *
 *  Created on: 07.09, 2018
 *      Author: Fabian Jenelten
 */

// motion generation utils
#include "motion_generation_utils/TrajectoryStateHandlerAngular.hpp"
#include "motion_generation_utils/conversions.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace motion_generation {

TrajectoryStateHandlerAngular::TrajectoryStateHandlerAngular():
    TrajectoryStateHandlerBase()
{
  optimizationState_.clear();
  optimizationState_.reserve(zmp::optimizationRotationalDofs.size());
  for(const auto& dim : zmp::optimizationRotationalDofs) {
    optimizationState_.push_back(dim);
  }
}

EulerAnglesZyx TrajectoryStateHandlerAngular::getAnglesZyxBaseToPlane() const {
  return EulerAnglesZyx(
      motionPlan_[zmp::CogDim::yaw].getPosition(),
      motionPlan_[zmp::CogDim::pitch].getPosition(),
      motionPlan_[zmp::CogDim::roll].getPosition());
}

EulerAnglesZyx TrajectoryStateHandlerAngular::getAnglesZyxBaseToPlaneAtTime(const double tk) const {
  return EulerAnglesZyx(
      motionPlan_[zmp::CogDim::yaw].getPositionAtTime(tk),
      motionPlan_[zmp::CogDim::pitch].getPositionAtTime(tk),
      motionPlan_[zmp::CogDim::roll].getPositionAtTime(tk));
}

EulerAnglesZyxDiff TrajectoryStateHandlerAngular::getEulerRatesZyxBaseInPlaneFrame() const {
  return EulerAnglesZyxDiff(
      motionPlan_[zmp::CogDim::yaw].getVelocity(),
      motionPlan_[zmp::CogDim::pitch].getVelocity(),
      motionPlan_[zmp::CogDim::roll].getVelocity());
}
EulerAnglesZyxDiff TrajectoryStateHandlerAngular::getEulerRatesZyxBaseInPlaneFrameAtTime(const double tk) const {
  return EulerAnglesZyxDiff(
      motionPlan_[zmp::CogDim::yaw].getVelocityAtTime(tk),
      motionPlan_[zmp::CogDim::pitch].getVelocityAtTime(tk),
      motionPlan_[zmp::CogDim::roll].getVelocityAtTime(tk));
}

EulerAnglesZyxDiff TrajectoryStateHandlerAngular::getEulerAccelerationZyxBaseInPlaneFrame() const {
  return EulerAnglesZyxDiff(
      motionPlan_[zmp::CogDim::yaw].getAcceleration(),
      motionPlan_[zmp::CogDim::pitch].getAcceleration(),
      motionPlan_[zmp::CogDim::roll].getAcceleration());
}
EulerAnglesZyxDiff TrajectoryStateHandlerAngular::getEulerAccelerationZyxBaseInPlaneFrameAtTime(const double tk) const {
  return EulerAnglesZyxDiff(
      motionPlan_[zmp::CogDim::yaw].getAccelerationAtTime(tk),
      motionPlan_[zmp::CogDim::pitch].getAccelerationAtTime(tk),
      motionPlan_[zmp::CogDim::roll].getAccelerationAtTime(tk));
}

LocalAngularVelocity TrajectoryStateHandlerAngular::getAngularVelocityBaseInPlaneFrame() const {
  return LocalAngularVelocity(
      zmp::getMapEulerAnglesZyxToAngularVelocityInInertialFrame(getAnglesZyxBaseToPlane().getUnique()) *
      getEulerRatesZyxBaseInPlaneFrame().toImplementation()
  );
}

LocalAngularVelocity TrajectoryStateHandlerAngular::getAngularVelocityBaseInPlaneFrameAtTime(const double tk) const {
  return LocalAngularVelocity(
      zmp::getMapEulerAnglesZyxToAngularVelocityInInertialFrame(getAnglesZyxBaseToPlaneAtTime(tk).getUnique()) *
      getEulerRatesZyxBaseInPlaneFrameAtTime(tk).toImplementation()
  );
}

AngularAcceleration TrajectoryStateHandlerAngular::getAngularAccelerationBaseInPlaneFrame() const {
  const auto anglesZyxBaseToPlane      = getAnglesZyxBaseToPlane().getUnique();
  const auto eulerRatesZyxInPlaneFrame = getEulerRatesZyxBaseInPlaneFrame();
  return AngularAcceleration(
      zmp::getMapEulerAnglesZyxToAngularVelocityInInertialFrame(anglesZyxBaseToPlane)*
      getEulerAccelerationZyxBaseInPlaneFrame().toImplementation() +
      zmp::getTimeDerivativeMapEulerAnglesZyxToAngularVelocityInInertialFrame(anglesZyxBaseToPlane, eulerRatesZyxInPlaneFrame)*
      eulerRatesZyxInPlaneFrame.toImplementation()
  );
}

AngularAcceleration TrajectoryStateHandlerAngular::getAngularAccelerationBaseInPlaneFrameAtTime(const double tk) const {
  const auto anglesZyxBaseToPlane      = getAnglesZyxBaseToPlaneAtTime(tk).getUnique();
  const auto eulerRatesZyxInPlaneFrame = getEulerRatesZyxBaseInPlaneFrameAtTime(tk);
  return AngularAcceleration(
      zmp::getMapEulerAnglesZyxToAngularVelocityInInertialFrame(anglesZyxBaseToPlane)*
      getEulerAccelerationZyxBaseInPlaneFrame().toImplementation() +
      zmp::getTimeDerivativeMapEulerAnglesZyxToAngularVelocityInInertialFrame(anglesZyxBaseToPlane, eulerRatesZyxInPlaneFrame)*
      eulerRatesZyxInPlaneFrame.toImplementation()
  );
}

bool TrajectoryStateHandlerAngular::addMotionPlanOrientation(
    const eulerAnglesZyxVector& anglesZyxPathToPlane,
    const eulerAnglesZyxDiffVector& angularRatesZyxPathInPlaneFrame,
    const std::vector<double>& duration,
    double optimizationHorizonInSeconds) {

  if (duration.size()+1u != anglesZyxPathToPlane.size() || anglesZyxPathToPlane.size() != angularRatesZyxPathInPlaneFrame.size()) {
    MELO_WARN_STREAM("[TrajectoryStateHandlerBase::setMotionPlan] Wrong vector dimension!");
    return false;
  }

  curves::SplineOptions opts;
  opts.acc0_  = 0.0;
  opts.accT_  = 0.0;

  for (auto splineId=0u; splineId<duration.size(); ++splineId) {
    for (const auto& dim : zmp::optimizationRotationalDofs) {
      opts.tf_    = duration[splineId];
      opts.pos0_  = anglesZyxPathToPlane[splineId].toImplementation()(zmp::toIndex(dim));
      opts.posT_  = anglesZyxPathToPlane[splineId+1].toImplementation()(zmp::toIndex(dim));
      opts.vel0_  = angularRatesZyxPathInPlaneFrame[splineId].toImplementation()(zmp::toIndex(dim));
      opts.velT_  = angularRatesZyxPathInPlaneFrame[splineId+1].toImplementation()(zmp::toIndex(dim));
      if(!addSpline(curves::PolynomialSplineQuintic(opts), dim)) { return false; }
    }
  }
  return true;
}


} /* namespace zmp */
