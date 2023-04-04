/*
 * TorsoStateDesired.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"

// STL
#include <memory>

namespace loco {

//! Desired state of the torso
/*! The desired state of the torso is defined by the desired position, linear velocity and linear acceleration of a
 * target point together with the desired orientation of the base frame and the angular velocity of the base.
 * The chosen target point is specified by the Enum TargetPoint.
 *
 */
class TorsoStateDesired {
 public:
  enum class TargetPoint : unsigned int { BASE = 0, WBCOM, WBCOMXY_BASEZ, TORSOCOM };

 public:
  TorsoStateDesired();
  virtual ~TorsoStateDesired() = default;

  TargetPoint getTargetPoint() const { return targetPoint_; }

  void setTargetPoint(TargetPoint targetPoint) { targetPoint_ = targetPoint; }

  const Position& getPositionControlToTargetInControlFrame() const;
  void setPositionControlToTargetInControlFrame(const Position& positionControlToBaseInControlFrame);

  const Position& getPositionWorldToBaseInWorldFrame() const;
  void setPositionWorldToBaseInWorldFrame(const Position& positionWorldToBaseInWorldFrame);

  const RotationQuaternion& getOrientationControlToBase() const;
  void setOrientationControlToBase(const RotationQuaternion& orientation);

  const EulerAnglesZyx& getOrientationEulerAnglesZyxControlToBase() const;

  void setOrientationEulerAnglesZyxBaseToWorld(const EulerAnglesZyx& orientationBaseToWorld);
  const EulerAnglesZyx& getOrientationEulerAnglesZyxBaseToWorld() const;

  void setLinearVelocityTargetInControlFrame(const LinearVelocity& linearVelocity);
  const LinearVelocity& getLinearVelocityTargetInControlFrame() const;

  void setAngularVelocityBaseInControlFrame(const LocalAngularVelocity& angularVelocity);
  const LocalAngularVelocity& getAngularVelocityBaseInControlFrame() const;

  void setLinearVelocityCommandedTargetInControlFrame(const LinearVelocity& linearVelocity);
  const LinearVelocity& getLinearVelocityCommandedTargetInControlFrame() const;

  void setAngularVelocityCommandedBaseInControlFrame(const LocalAngularVelocity& angularVelocity);
  const LocalAngularVelocity& getAngularVelocityCommandedBaseInControlFrame() const;

  void setLinearAccelerationTargetInControlFrame(const LinearAcceleration& linearAcceleration);
  const LinearAcceleration& getLinearAccelerationTargetInControlFrame() const;

  void setAngularAccelerationTargetInControlFrame(const AngularAcceleration& angularAcceleration);
  const AngularAcceleration& getAngularAccelerationTargetInControlFrame() const;

  const Position& getPositionErrorInControlFrame() const;
  void setPositionErrorInControlFrame(const Position& positionErrorInControlFrame);

  const LinearVelocity& getLinearVelocityErrorInControlFrame() const;
  void setLinearVelocityErrorInControlFrame(const LinearVelocity& linearVelocityErrorInControlFrame);

  const LocalAngularVelocity& getAngularVelocityErrorInControlFrame() const;
  void setAngularVelocityErrorInControlFrame(const LocalAngularVelocity& angularVelocityErrorInControlFrame);

  void setDesiredPositionOffsetInWorldFrame(const Position& positionTargetOffsetInWorldFrame);
  void setDesiredOrientationOffset(const RotationQuaternion& orientationOffset);

  const Position& getDesiredPositionOffsetInWorldFrame() const;
  const RotationQuaternion& getDesiredOrientationOffset() const;

  friend std::ostream& operator<<(std::ostream& out, const TorsoStateDesired& desiredState);

 protected:
  TargetPoint targetPoint_;
  Position positionControlToTargetInControlFrame_;
  Position positionWorldToBaseInWorldFrame_;

  RotationQuaternion orientationControlToBase_;
  EulerAnglesZyx orientationEulerAnglesZyxControlToBase_;
  EulerAnglesZyx orientationEulerAnglesZyxBaseToWorld_;

  LinearVelocity linearVelocityTargetInControlFrame_;
  LocalAngularVelocity angularVelocityBaseInControlFrame_;

  LinearVelocity linearVelocityCommandedTargetInControlFrame_;
  LocalAngularVelocity angularVelocityCommandedBaseInControlFrame_;

  LinearAcceleration linearAccelerationTargetInControlFrame_;
  AngularAcceleration angularAccelerationTargetInControlFrame_;

  Position positionErrorInControlFrame_;
  LinearVelocity linearVelocityErrorInControlFrame_;
  LocalAngularVelocity angularVelocityErrorInControlFrame_;

  //! The position offset is added to the desired position.
  Position desiredPositionOffsetInWorldFrame_;

  //! The orientation offset is added to the desired orientation.
  RotationQuaternion desiredOrientationOffset_;
};

using TorsoStateDesiredPtr = std::unique_ptr<TorsoStateDesired>;

} /* namespace loco */
