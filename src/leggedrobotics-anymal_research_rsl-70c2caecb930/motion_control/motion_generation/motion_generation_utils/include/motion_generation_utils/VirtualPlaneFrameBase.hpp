/*
 * VirtualPlaneFrameBase.hpp
 *
 *  Created on: June 01, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// motion generation utils
#include "motion_generation_utils/typedefs.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

namespace motion_generation {

class VirtualPlaneFrameBase {
 public:
  VirtualPlaneFrameBase();
  virtual ~VirtualPlaneFrameBase() = default;

  //! Project position in world frame along desirable axis onto virtual plane. The projection is given in world frame.
  motion_generation::Position projectOntoVirtualPlaneInWorldFrame(
      const motion_generation::Position& positionWorldToPointInWorldFrame,
      const motion_generation::Vector& projectionAxisInWorldFrame = motion_generation::Vector::UnitZ()) const;

  //! Project position in world frame along plane normal onto virtual plane. The projection is given in world frame.
  motion_generation::Position projectOntoVirtualPlaneAlongPlaneNormalInWorldFrame(const motion_generation::Position& positionWorldToPointInWorldFrame) const;

  //! Get plane normal of virtual plane in world frame.
  const motion_generation::Vector& getPlaneNormalInWorldFrame() const;

  //! Get plane normal of virtual plane in plane frame.
  const motion_generation::Vector& getPlaneNormalInPlaneFrame() const;

  //! Get the pose of the plane w.r.t. the world frame.
  const motion_generation::Pose& getPosePlaneToWorld() const;

  //! Copy protected class members.
  virtual void copy(const VirtualPlaneFrameBase& virtualPlane);

  //! Set zero position and identity rotation.
  virtual void setIdentity();

 protected:
  //! The pose of the virtual plane frame w.r.t. the world frame.
  motion_generation::Pose poseVirtualPlaneToWorld_;

  //! Plane normal in world frame.
  motion_generation::Vector planeNormalInWorldFrame_;

  //! Plane normal in plane frame.
  motion_generation::Vector planeNormalInPlaneFrame_;
};


} /* namespace motion_gen */
