/*!
 * @file     AnymalState.hpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Dec, 2014
 */

#pragma once

// anymal model
#include "anymal_model/typedefs.hpp"

// romo
#include <romo/ExtendedRobotState.hpp>

// anymal description
#include <anymal_description/AnymalDescription.hpp>
#include <anymal_description/AnymalTopology.hpp>

// stl
#include <ostream>
#include <unordered_map>

namespace anymal_model {

//! Anymal State
/*! This class cannot have any dynamically allocated memory, since this object is used in shared memory.
 */
class AnymalState : public romo::ExtendedRobotState<CAD> {
 private:
  using Base = romo::ExtendedRobotState<CAD>;

 public:
  AnymalState() = default;
  ~AnymalState() override = default;

  void getPoseBaseToWorld(kindr::HomTransformQuatD& poseBaseToWorld) const;

  void setPoseBaseToWorld(const kindr::HomTransformQuatD& poseBaseToWorld);

  /*! Sets this state equal to the linearly interpolated state between state0 and state1.
   * @param t           interpolation parameter [0, 1]
   * @param state0      state at t=0
   * @param state1      state at t=1
   */
  virtual void setGeneralizedCoordinatesToLinearlyInterpolated(double t, const AnymalState& state0, const AnymalState& state1);

  /*! @returns this state with an added small delta to a specific state.
   *
   * This operator can be used for finite-difference method.
   *
   * @param delta     value that is added
   * @param uIndex    index of the generalized state.
   */
  virtual AnymalState boxPlus(double delta, unsigned int uIndex, bool useQuaternion) const;

  const Pose& getFrameTransform(const AT::FrameTransformEnum& transformEnum) const;
  void setFrameTransform(const AT::FrameTransformEnum& transformEnum, const Pose& pose);

  friend std::ostream& operator<<(std::ostream& out, const AnymalState& state);

 protected:
  Pose poseFootprintToOdom_;
  Pose poseFeetcenterToOdom_;
};

}  // namespace anymal_model
