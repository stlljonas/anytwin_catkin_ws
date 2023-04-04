/*
 * TorsoPropertiesBase.hpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"

// STL
#include <memory>

namespace loco {

class TorsoProperties {
 public:
  TorsoProperties();
  virtual ~TorsoProperties() = default;

  virtual bool initialize(double dt) = 0;
  virtual bool advance(double dt) = 0;

  virtual double getMass() const;
  virtual void setMass(double mass);

  virtual const Eigen::Matrix3d& getInertiaTensorInBaseFrame() const;
  virtual void setInertiaTensorInBaseFrame(const Eigen::Matrix3d& inertiaTensorInBaseFrame);

  virtual const Position& getBaseToCenterOfMassPositionInBaseFrame() const;
  virtual void setBaseToCenterOfMassPositionInBaseFrame(const Position& centerOfMassInBaseFrame);

  const LinearAcceleration& getGravity() const;
  void setGravity(const LinearAcceleration& gravity);

  const Vector& getGravityAxisInWorldFrame();
  const Vector& getHeadingAxisInBaseFrame();
  const Vector& getLateralAxisInBaseFrame();
  const Vector& getVerticalAxisInBaseFrame();

  void setHeadingAxisInBaseFrame(const Vector& axis);
  void setLateralAxisInBaseFrame(const Vector& axis);
  void setVerticalAxisInBaseFrame(const Vector& axis);

  void setMaximumBaseTwistInControlFrame(const Twist& maximumBaseTwistInControlFrame);
  const Twist& getMaximumBaseTwistInControlFrame() const;

 protected:
  //! Gravitational acceleration
  LinearAcceleration gravity_;
  //! Gravity axis in world frame
  Vector gravityAxisInWorldFrame_;
  //! Torso masse
  double mass_;
  //! CoM positon in base frame
  Position positionBaseToCenterOfMassInBaseFrame_;
  //! Torso heading axis
  Vector headingAxisInBaseFrame_;
  //! Torso lateral axis
  Vector lateralAxisInBaseFrame_;
  //! Torso vertical axis
  Vector verticalAxisInBaseFrame_;
  //! Torso inertia matrix
  Eigen::Matrix3d inertiaTensorInBaseFrame_;
  // Max velocities
  Twist maximumBaseTwistInControlFrame_;
};

using TorsoPropertiesPtr = std::unique_ptr<TorsoProperties>;

} /* namespace loco */
