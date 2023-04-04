/*
 * LimbStateMeasured.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/joints/MeasuredJointStates.hpp"
#include "loco/common/typedefs.hpp"

// STL
#include <memory>

namespace loco {

class LimbStateMeasured : public MeasuredJointStates {
 public:
  explicit LimbStateMeasured(const unsigned int numDofLimb);
  ~LimbStateMeasured() override = default;

  virtual const Position& getPositionWorldToLimbBaseInWorldFrame() const;
  virtual void setPositionWorldToLimbBaseInWorldFrame(const Position& positionWorldToLimbBaseInWorldFrame);

  virtual const Position& getPositionWorldToLimbBaseInBaseFrame() const;
  virtual void setPositionWorldToLimbBaseInBaseFrame(const Position& positionWorldToLimbBaseInBaseFrame);

  virtual const Position& getPositionBaseToLimbBaseInBaseFrame() const;
  virtual void setPositionBaseToLimbBaseInBaseFrame(const Position& positionBaseToLimbBaseInBaseFrame);

  virtual const LinearVelocity& getLinearVelocityLimbBaseInWorldFrame() const;
  virtual void setLinearVelocityLimbBaseInWorldFrame(const LinearVelocity& linearVelocityLimbBaseInWorldFrame);

 protected:
  //! The position of the limb attachment point relative to the world frame expressed in the world frame.
  Position positionWorldToLimbBaseInWorldFrame_;

  //! The position of the limb attachment point relative to the base frame expressed in the world frame.
  Position positionWorldToLimbBaseInBaseFrame_;

  //! The position of the limb attachment point relative to the base frame expressed in the base frame.
  Position positionBaseToLimbBaseInBaseFrame_;

  //! The absolute linear velocity of the limb attachment point expressed in the world frame.
  LinearVelocity linearVelocityLimbBaseInWorldFrame_;
};

using LimbStateMeasuredPtr = std::unique_ptr<LimbStateMeasured>;

} /* namespace loco */
