/*
 * TorsoStateMeasured.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/torso/TorsoStateMeasuredInControlFrame.hpp"
#include "loco/common/typedefs.hpp"

// STL
#include <memory>

namespace loco {

//! Measured state of the torso
class TorsoStateMeasured {
 public:
  TorsoStateMeasured();
  virtual ~TorsoStateMeasured() = default;

  const Position& getPositionWorldToBaseInWorldFrame() const;
  void setPositionWorldToBaseInWorldFrame(const Position& position);

  const Position& getPositionWorldToCenterOfMassInWorldFrame() const;
  void setPositionWorldToCenterOfMassInWorldFrame(const Position& positionWorldToCenterOfMassInWorldFrame);

  const LinearVelocity& getLinearVelocityBaseInBaseFrame() const;
  void setLinearVelocityBaseInBaseFrame(const LinearVelocity& linearVelocity);

  const LinearVelocity& getLinearVelocityCenterOfMassInBaseFrame() const;
  void setLinearVelocityCenterOfMassInBaseFrame(const LinearVelocity& linearVelocityCenterOfMassInBaseFrame);

  const LocalAngularVelocity& getAngularVelocityBaseInBaseFrame() const;
  void setAngularVelocityBaseInBaseFrame(const loco::LocalAngularVelocity& angularVelocity);

  const RotationQuaternion& getOrientationWorldToBase() const;
  void setOrientationWorldToBase(const RotationQuaternion& orientation);

  const EulerAnglesZyx& getOrientationEulerAnglesZyxBaseToWorld() const;

  const TorsoStateMeasuredInControlFrame& inControlFrame() const;
  TorsoStateMeasuredInControlFrame& inControlFrame();

  friend std::ostream& operator<<(std::ostream& out, const TorsoStateMeasured& state);

 protected:
  TorsoStateMeasuredInControlFrame stateInControlFrame_;

  Position positionWorldToBaseInWorldFrame_;
  Position positionWorldToCenterOfMassInWorldFrame_;
  RotationQuaternion orientationWorldToBase_;
  EulerAnglesZyx orientationEulerAnglesZyxBaseToWorld_;

  LinearVelocity linearVelocityBaseInBaseFrame_;
  LinearVelocity linearVelocityCenterOfMassInBaseFrame_;
  LocalAngularVelocity angularVelocityBaseInBaseFrame_;
};

using TorsoStateMeasuredPtr = std::unique_ptr<TorsoStateMeasured>;

} /* namespace loco */
