/*
 * SwingTrajectoryGeneratorSplineOptimized.hpp
 *
 *  Created on: Jan. 15, 2017
 *      Author: Fabian Jenelten
 */
#pragma once

 //loco
#include <loco/swing_trajectory_generation/SwingTrajectoryGeneratorBase.hpp>
#include <loco/swing_trajectory_generation/SwingTrajectoryGeneratorModule.hpp>
#include <loco/common/WholeBody.hpp>
#include <loco/common/TerrainModelBase.hpp>
#include <loco/gait_pattern/contact_schedules.hpp>

// swing trajectory generation
#include <swing_trajectory_generation/SwingTrajectoryPlanner.hpp>


// curves
#include "curves/PolynomialSplineScalarCurve.hpp"

#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco {

namespace stg {

/*
 * Optimization status:
 *  > success: swing trajectory is available
 *  > no_plan_available: First optimization has not yet completed
 *  > end_of_container: Swing trajectory is available, but we reached the end of the container time
 *  > init_failed: Failed to set container time after optimization has completed
 */
CONSECUTIVE_ENUM(OptStatus, success, no_plan_available, end_of_container, init_failed)

} /* namespace stg */

class SwingTrajectoryGeneratorSplineOptimized : public SwingTrajectoryGeneratorModule {

 public:
  SwingTrajectoryGeneratorSplineOptimized(
      WholeBody& wholeBody,
      TerrainModelBase& terrain,
      contact_schedule::ContactScheduleAnymalBase& contactSchedule);
  ~SwingTrajectoryGeneratorSplineOptimized() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;
  bool initialize(double dt) override;
  bool stop() override;
  bool advance(double dt) override;

  //! Get optimized foothold state.
  bool getDesiredFootState(
      Position& positionWorldToDesiredFootInWorldFrame,
      LinearVelocity& linearVelocityDesiredFootInWorldFrame,
      LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
      const Position& positionWorldToDesiredFootholdInControlFrame,
      LegBase* leg, double dt) override;

  // Returns a vector of motions plans.
  void getMotionPlan(std::vector<sto::MotionPlan>& motionPlan) const;

 protected:
  //! Runs the optimization.
  bool planTrajectory();

  //! Fills in all informations into the motion plan required for the optimization.
  bool setMotionPlan(sto::MotionPlan& motionPlan, unsigned int legId);

  //! Reads and advances the motion plan.
  bool processReferenceSignalsForLeg(double timeToAdvance, unsigned int legId);

  //! The whole body.
  WholeBody& wholeBody_;

  //! The terrain.
  TerrainModelBase& terrain_;

  //! Gait Pattern
  contact_schedule::ContactScheduleAnymalBase& contactSchedule_;

  //! Swing trajectory planner.
  SwingTrajectoryPlanner swingTrajectoryOptimizer_;

  //! A mutex to synchronize access to the motion plan.
  mutable boost::shared_mutex mutexMotionPlan_;

  //! Vector of motion plan (passes signals to the optimizer, each leg has its own motion plan).
  std::vector<sto::MotionPlan> motionPlan_;

  //! Desired final height in plane frame at foot touch-down.
  double finalHeightPositionOffset_;

  //! Desired final z-velocity in plane frame at foot touch-down.
  double finalHeightVelocityOffset_;

  //! If true, the trajectory is initialized with the current measured foot state, otherwise with the previous stance leg state.
  bool initWithMeasuredFootState_;

  //! If true, debugging information is displayed to the console.
  bool verbose_;

  //! Status of the optimization for each leg.
  std::vector<stg::OptStatus> optStatus_;

  //! If swing phase is larger than this threshold, the optimization is skipped (avoid wrong contact detection).
  double upperSwingPhaseThreshold_;
  double lowerSwingTimeThreshold_;

  //! Vector of desired footholds in world frame.
  std::vector<Position, Eigen::aligned_allocator<Position>> positionWorldToDesiredFootInWorldFrame_;

  //! Vector of desired foothold velocities in world frame.
  std::vector<LinearVelocity, Eigen::aligned_allocator<LinearVelocity>> linearVelocityDesiredFootInWorldFrame_;

  //! Vector of desired foothold accelerations in world frame.
  std::vector<LinearAcceleration, Eigen::aligned_allocator<LinearAcceleration>> linearAccelerationDesiredFootInWorldFrame_;
};

} /* namespace loco */
