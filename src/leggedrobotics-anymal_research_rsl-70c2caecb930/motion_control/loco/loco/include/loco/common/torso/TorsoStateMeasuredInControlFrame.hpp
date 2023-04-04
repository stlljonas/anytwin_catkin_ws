/*
 * TorsoStateMeasuredInControlFrame.hpp
 *
 *  Created on:  May 18, 2016
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"

namespace loco {

//! State of the torso expressed in control frame
/*! The control frame combines information about the state of the robot w.r.t. the world and w.r.t. the terrain.
 *  For this reason, the state is update after or by the the terrain perception.
 */
class TorsoStateMeasuredInControlFrame {
 public:
  TorsoStateMeasuredInControlFrame();
  virtual ~TorsoStateMeasuredInControlFrame() = default;

  const Position& getPositionControlToBaseInControlFrame() const;
  void setPositionControlToBaseInControlFrame(const Position& positionControlToBaseInControlFrame);

  const RotationQuaternion& getOrientationWorldToControl() const;
  void setOrientationWorldToControl(const RotationQuaternion& orientation);

  const RotationQuaternion& getOrientationControlToBase() const;
  void setOrientationControlToBase(const RotationQuaternion& orientation);

  const Position& getPositionWorldToControlInWorldFrame() const;
  void setPositionWorldToControlInWorldFrame(const Position& positionWorldToControlInWorldFrame);

  const LinearVelocity& getLinearVelocityBaseInControlFrame() const;
  void setLinearVelocityBaseInControlFrame(const LinearVelocity& linearVelocity);

  const LocalAngularVelocity& getAngularVelocityBaseInControlFrame() const;
  void setAngularVelocityBaseInControlFrame(const loco::LocalAngularVelocity& angularVelocity);

  const LocalAngularVelocity& getAngularVelocityControlInWorldFrame() const;
  void setAngularVelocityControlInWorldFrame(const loco::LocalAngularVelocity& angularVelocity);

  const EulerAnglesZyx& getOrientationEulerAnglesZyxControlToBase() const;

  friend std::ostream& operator<<(std::ostream& out, const TorsoStateMeasuredInControlFrame& state);

 protected:
  RotationQuaternion orientationWorldToControl_;
  RotationQuaternion orientationControlToBase_;
  EulerAnglesZyx orientationEulerAnglesZyxControlToBase_;
  Position positionWorldToControlInWorldFrame_;
  Position positionControlToBaseInControlFrame_;
  LinearVelocity linearVelocityBaseInControlFrame_;
  LocalAngularVelocity angularVelocityBaseInControlFrame_;
  LocalAngularVelocity angularVelocityControlInWorldFrame_;
};

} /* namespace loco */
