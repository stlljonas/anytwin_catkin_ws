/*
 * MotionPlan.hpp
 *
 *  Created on: Jan. 15, 2017
 *      Author: Fabian Jenelten
 */
#pragma once


// motion generation utils
#include "motion_generation_utils/TrajectoryStateHandlerLineSearchLinear.hpp"
#include "motion_generation_utils/MotionPlanLinear.hpp"

// swing trajectory generation
#include "swing_trajectory_generation/VirtualPlaneFrameFoot.hpp"


namespace sto {

class MotionPlan: virtual public motion_generation::MotionPlanLinear {
 public:

  MotionPlan();
  ~MotionPlan() override = default;

  bool initialize();

  //! Reset the motion plan.
  bool resetSwingTrajectory();

  //! Set virtual plane Frame by two points that have to lie on that plane.
  bool setVirtualPlaneFrame(
      const motion_generation::Position& positionWorldToPreviousStanceFootholdInWorldFrame,
      const motion_generation::RotationQuaternion& orientationWorldToControl);

  /* ! Set timing informations
   * @param swingDuration   duration from stance to stance
   * @timeSpentInSwing      time from previous stance to current end effector location
   * @isFootLiftOf          true if current optimization is the first one since foot lift-off.
   */
  void setTimingInformations(
      double swingDuration,
      double stanceDuration,
      double timeSpendInSwing,
      double trajectoryStartTime,
      bool isPreviousOptimizationAvailable_);

  //! Returns a pointer to the swing trajectory.
  motion_generation::TrajectoryStateHandlerLineSearchLinear* getSwingTrajectoryInPlaneFramePtr();

  //! Returns the swing trajectory.
  const motion_generation::TrajectoryStateHandlerLineSearchLinear& getSwingTrajectoryInPlaneFrame() const;

  //! Get vector of spline durations.
  const std::vector<double>& getSwingSplineDurations() const;

  //! Set vector of spline durations.
  void setSplineDurations(std::vector<double>& swingSplineDurations, unsigned int numOfTotSwingSplines);

  //! Returns the spline index valid at some time where the spine index is computed w.r.t. to a trajectory from stance to stance.
  unsigned int getActiveSplineId(double time) const;

  //! Returns the spline index valid for time=0 where the spine index is computed w.r.t. to a trajectory from stance to stance.
  unsigned int getSplineIdOffset() const;

  //! Returns the swing duration of a leg.
  double getSwingDuration() const;

  //! Returns stance duration of a leg.
  double getSanceDuration() const;

  //! Returns time spend in swing of a leg.
  double getTimeSpendInSwing() const;

  //! Returns starting time of trajectory (time w.r.t. previous foot lift-off event).
  double getTrajectoryStartTime() const;

  //! Get optimization trajectory state in world frame.
  void getPositionWorldToDesiredFootInWorldFrame(motion_generation::Position& positionWorldToDesiredFootInWorldFrame) const;
  void getLinearVelocityDesiredFootInWorldFrame(motion_generation::LinearVelocity& linearVelocityDesiredFootInWorldFrame) const;
  void getLinearAccelerationDesiredFootInWorldFrame(motion_generation::LinearAcceleration& linearAccelerationDesiredFootInWorldFrame) const;

  //! Get optimized trajectory state at time in world frame.
  void getPositionWorldToDesiredFootInWorldFrameAtTime(motion_generation::Position& positionWorldToDesiredFootInWorldFrame, double dt) const;
  void getLinearVelocityDesiredFootInWorldFrameAtTime(motion_generation::LinearVelocity& linearVelocityDesiredFootInWorldFrame, double dt) const;
  void getLinearAccelerationDesiredFootInWorldFrameAtTime(motion_generation::LinearAcceleration& linearAccelerationDesiredFootInWorldFrame, double dt) const;

  //! Returns the container time valid for all splines.
  double getContainerTime() const;

  //! Returns the container duration valid for all splines.
  double getContainerDuration() const;

  //! Compute and update container duration based on the spline times.
  bool updateContainerDuration();

  //! If true, the optimization is the first after the foot has lift off.
  bool isPreviousOptimizationAvailable();

  //! Only copy objects from the previous optimization that are required for the next optimization.
  void copy(const MotionPlan& motionPlan);

  //! Add stance trajectory to optimized swing trajectory.
  bool addStancePhase();

 protected:

  //! Optimized swing trajectory.
  motion_generation::TrajectoryStateHandlerLineSearchLinear swingTrajectoryInPlaneFrame_;

  //! Number of splines used for the entire swing trajectory.
  unsigned int numOfTotSwingSplines_;

  //! Number of splines used for the entire stance trajectory.
  unsigned int numOfTotStanceSplines_;

  //! Duration of indifidual spline segments.
  std::vector<double> swingSplineDurations_;

  //! Total swing duration of a leg.
  double swingDuration_;

  //! Total stance duration of a leg.
  double stanceDuration_;

  //! Time spend in swing for a leg.
  double timeSpendInSwing_;

  //! Time w.r.t. foot lift-off event.
  double trajectoryStartTime_;

  //! If true, a previous optimization is available.
  bool isPreviousOptimizationAvailable_;
};

} /* namespace sto */
