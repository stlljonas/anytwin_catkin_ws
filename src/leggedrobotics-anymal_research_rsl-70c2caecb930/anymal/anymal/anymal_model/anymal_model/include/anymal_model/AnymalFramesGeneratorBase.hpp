/*
 * AnymalFramesGeneratorBase.hpp
 *
 *  Created on: Aug 3, 2018
 *      Author: Fabian Tresoldi
 */

#pragma once

// romo
#include <romo/RobotModel.hpp>

// kindr
#include <kindr/Core>

namespace anymal_model {

//! Generates coordinate frame based on the state of the anymal.
template <typename ConcreteAnymalDescription_, typename AnymalState_>
class AnymalFramesGeneratorBase {
 public:
  using RobotModel = romo::RobotModel<ConcreteAnymalDescription_, AnymalState_>;

  AnymalFramesGeneratorBase() = default;
  virtual ~AnymalFramesGeneratorBase() = default;

  void reset() {
    resetBase();
    resetDerived();
  }

  void resetBase() {
    poseFootprintToOdom_.setIdentity();
    poseFeetcenterToOdom_.setIdentity();
  }

  virtual void resetDerived() = 0;

  /*! Updates the frames
   * @param model with the state of the robot and the contact states
   */
  virtual void update(const RobotModel& model) = 0;

  const kindr::HomTransformQuatD& getPoseFootprintToOdom() const { return poseFootprintToOdom_; };
  const kindr::HomTransformQuatD& getPoseFeetcenterToOdom() const { return poseFeetcenterToOdom_; };

 protected:
  kindr::HomTransformQuatD poseFootprintToOdom_;
  kindr::HomTransformQuatD poseFeetcenterToOdom_;
};

}  // namespace anymal_model
