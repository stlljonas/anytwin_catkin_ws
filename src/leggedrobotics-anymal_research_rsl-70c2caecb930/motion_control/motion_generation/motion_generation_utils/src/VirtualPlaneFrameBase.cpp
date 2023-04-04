/*
 * VirtualPlaneFrameBase.hpp
 *
 *  Created on: June 01, 2018
 *      Author: Fabian Jenelten
 */

// motion generation utils
#include "motion_generation_utils/VirtualPlaneFrameBase.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace motion_generation {

VirtualPlaneFrameBase::VirtualPlaneFrameBase():
    poseVirtualPlaneToWorld_(),
    planeNormalInWorldFrame_(motion_generation::Vector::UnitZ()),
    planeNormalInPlaneFrame_(motion_generation::Vector::UnitZ())
{

}

motion_generation::Position VirtualPlaneFrameBase::projectOntoVirtualPlaneInWorldFrame(
    const motion_generation::Position& positionWorldToPointInWorldFrame,
    const motion_generation::Vector& projectionAxisInWorldFrame) const {

  /*
   * If P is the Projection of point Q along projection axis c, then
   *  n*(P-K) = 0,
   * where P and K are points on the plane.
   * Since P is also the projection of Q, it holds that
   *  n*(Q+c*lambda-K) = 0
   *  lambda = -n*(Q-K)/(n*c)
   */

  const double axisOnPlane = planeNormalInWorldFrame_.dot(projectionAxisInWorldFrame);

  if (axisOnPlane == 0.0) {
    MELO_FATAL_STREAM("[VirtualPlaneFrameBase::projectOntoVirtualPlaneInWorldFrame] Projection axis lies in plane.");
    return positionWorldToPointInWorldFrame;
  }

  const double lambda = -planeNormalInWorldFrame_.toImplementation().dot(
      (positionWorldToPointInWorldFrame - poseVirtualPlaneToWorld_.getPosition()).toImplementation()) / axisOnPlane;
  return (positionWorldToPointInWorldFrame + motion_generation::Position(lambda*projectionAxisInWorldFrame));
}

motion_generation::Position VirtualPlaneFrameBase::projectOntoVirtualPlaneAlongPlaneNormalInWorldFrame(
    const motion_generation::Position& positionWorldToPointInWorldFrame) const {
  return projectOntoVirtualPlaneInWorldFrame(positionWorldToPointInWorldFrame, planeNormalInWorldFrame_);
}

const motion_generation::Vector& VirtualPlaneFrameBase::getPlaneNormalInWorldFrame() const {
  return planeNormalInWorldFrame_;
}

const motion_generation::Vector& VirtualPlaneFrameBase::getPlaneNormalInPlaneFrame() const {
  return planeNormalInPlaneFrame_;
}

const motion_generation::Pose& VirtualPlaneFrameBase::getPosePlaneToWorld() const {
  return poseVirtualPlaneToWorld_;
}

void VirtualPlaneFrameBase::copy(const VirtualPlaneFrameBase& virtualPlane) {
  poseVirtualPlaneToWorld_ = virtualPlane.getPosePlaneToWorld();
  planeNormalInWorldFrame_ = virtualPlane.getPlaneNormalInWorldFrame();
}

void VirtualPlaneFrameBase::setIdentity() {
  poseVirtualPlaneToWorld_.setIdentity();
  planeNormalInWorldFrame_ = motion_generation::Vector::UnitZ();
  planeNormalInPlaneFrame_ = motion_generation::Vector::UnitZ();
}

} /* namespace motion_gen */
