/*
 * TrajectoryStateHandlerLinear.hpp
 *
 *  Created on: 07.09, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

//zmp
#include "motion_generation_utils/TrajectoryStateHandlerBase.hpp"

namespace motion_generation {

class TrajectoryStateHandlerLinear : virtual public TrajectoryStateHandlerBase {
public:

  TrajectoryStateHandlerLinear();
  ~TrajectoryStateHandlerLinear() override = default;

  //! Returns actual CoG state (position state).
  void getPositionPlaneToComInPlaneFrame(Position& ComStateInPlaneFrame) const;
  Position getPositionPlaneToComInPlaneFrame() const;

  //! Returns actual CoG state (velocity state).
  void getLinearVelocityComInPlaneFrame(LinearVelocity& ComStateInPlaneFrame) const;
  LinearVelocity getLinearVelocityComInPlaneFrame() const;

  //! Returns actual CoG state (acceleration state).
  void getLinearAccelerationComInPlaneFrame(LinearAcceleration& ComStateInPlaneFrame) const;
  LinearAcceleration getLinearAccelerationComInPlaneFrame() const;

  //! Returns CoG state at time (position state).
  void getPositionPlaneToComInPlaneFrameAtTime(Position& ComStateInPlaneFrame, double tk) const;
  Position getPositionPlaneToComInPlaneFrameAtTime(const double tk) const;

  //! Returns CoG state at time (velocity state).
  void getLinearVelocityComInPlaneFrameAtTime(LinearVelocity& ComStateInPlaneFrame, double tk) const;
  LinearVelocity getLinearVelocityComInPlaneFrameAtTime(const double tk) const;

  //! Returns CoG state at time (acceleration state).
  void getLinearAccelerationComInPlaneFrameAtTime(LinearAcceleration& ComStateInPlaneFrame, double tk) const;
  LinearAcceleration getLinearAccelerationComInPlaneFrameAtTime(const double tk) const;

  //! Set motion plan using knot points (angular part is set to zero).
  bool setMotionPlanLinear(
      const positionVector& positionKnotInPlaneFrame,
      const velocityVector& velocityKnotInPlaneFrame,
      const accelerationVector& accelerationKnotInPlaneFrame,
      const std::vector<double>& duration,
      double optimizationHorizonInSeconds);

protected:

  //! Add motion plan (linear part) using knot points.
  bool addMotionPlanTranslational(
      const positionVector& positionKnotInPlaneFrame,
      const velocityVector& velocityKnotInPlaneFrame,
      const accelerationVector& accelerationKnotInPlaneFrame,
      const std::vector<double>& duration,
      double optimizationHorizonInSeconds);

};

} /* namespace motion_generation */
