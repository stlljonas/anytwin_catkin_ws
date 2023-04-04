/*
 * EndEffectorStateDesired.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/joints/DesiredJointStates.hpp>
#include <loco/common/typedefs.hpp>

// STL
#include <memory>

namespace loco {

class EndEffectorStateBase {
 public:
  EndEffectorStateBase();
  virtual ~EndEffectorStateBase() = default;

  // World frame.
  void setPositionWorldToEndEffectorInWorldFrame(const Position& positionWorldToEndEffectorInWorldFrame);
  const Position& getPositionWorldToEndEffectorInWorldFrame() const;

  void setLinearVelocityEndEffectorInWorldFrame(const LinearVelocity& linearVelocityEndEffectorInWorldFrame);
  const LinearVelocity& getLinearVelocityEndEffectorInWorldFrame() const;

  void setLinearAccelerationEndEffectorInWorldFrame(const LinearAcceleration& linearAccelerationEndEffectorInWorldFrame);
  const LinearAcceleration& getLinearAccelerationEndEffectorInWorldFrame() const;

  void setOrientationWorldToEndEffector(const RotationQuaternion& orientationWorldToEndEffector);
  const RotationQuaternion& getOrientationWorldToEndEffector() const;

  void setAngularVelocityEndEffectorInWorldFrame(const LocalAngularVelocity& angularVelocityEndEffectorInWorldFrame);
  const LocalAngularVelocity& getAngularVelocityEndEffectorInWorldFrame() const;

  void setAngularAccelerationEndEffectorInWorldFrame(const AngularAcceleration& angularAccelerationEndEffectorInWorldFrame);
  const AngularAcceleration& getAngularAccelerationEndEffectorInWorldFrame() const;

  const Force& getForceAtEndEffectorInWorldFrame() const;
  void setForceAtEndEffectorInWorldFrame(const Force& forceAtEndEffectorInWorldFrame);

  const Torque& getTorqueAtEndEffectorInWorldFrame() const;
  void setTorqueAtEndEffectorInWorldFrame(const Torque& torqueAtEndEffectorInWorldFrame);

 protected:
  // World frame.
  Position positionWorldToEndEffectorInWorldFrame_;

  LinearVelocity linearVelocityEndEffectorInWorldFrame_;
  LinearAcceleration linearAccelerationEndEffectorInWorldFrame_;

  RotationQuaternion orientationWorldToEndEffector_;
  LocalAngularVelocity angularVelocityEndEffectorInWorldFrame_;
  AngularAcceleration angularAccelerationEndEffectorInWorldFrame_;

  Force forceAtEndEffectorInWorldFrame_;
  Torque torqueAtEndEffectorInWorldFrame_;
};

using EndEffectorStateBasePtr = std::unique_ptr<EndEffectorStateBase>;

} /* namespace loco */
