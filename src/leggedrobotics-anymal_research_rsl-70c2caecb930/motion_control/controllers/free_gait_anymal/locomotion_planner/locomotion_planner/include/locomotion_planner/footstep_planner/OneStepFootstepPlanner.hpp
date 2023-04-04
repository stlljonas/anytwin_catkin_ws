/*
 * OneStepFootstepPlanner.hpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "locomotion_planner/common/type_defs.hpp"
#include "locomotion_planner/common/Parameters.hpp"
#include "locomotion_planner/gait_pattern/GaitPatternBase.hpp"
#include "locomotion_planner/footstep_planner/FootstepPlannerBase.hpp"

#include <free_gait_core/free_gait_core.hpp>
#include <Eigen/Core>
#include <robot_utils/surfaces/Ellipsoid.hpp>

#include <memory>

namespace locomotion_planner {

class OneStepFootstepPlanner : public FootstepPlannerBase
{
 public:
  OneStepFootstepPlanner(const free_gait::AdapterBase& adapter, Parameters& parameters, GaitPatternBase& gaitPattern);
  virtual ~OneStepFootstepPlanner();

  bool plan(const Pose& goalPose, const Transform& goalToWorldFrameTransform, const double speedFactor,
            const bool isNewGoal, const bool allowIntermediatePoses, std::vector<Step>& plan, bool& lastStepToGoal);
  double getExpectedUpperPlanningTime() const;
  void stop();

 private:
  void computeIntermediateGoals();
  void determineStartFootprintAndPose();
  void computePoseDifference();
  void updateStepPose(const bool tentative);
  bool determineNextSwingLeg(const bool directionChanged);
  bool determineDirection();
  const free_gait::Footstep computeNextFootstep();

  Parameters& parameters_;

  Transform goalToWorldFrameTransform_;
  Pose startPose_; // In world frame.
  std::vector<Pose> goalPoses_; // In goal frame.
  double speedFactor_;
  PlanarPose goalPoseInStartFrame_;
  PlanarPose currentStepPose_; // In start pose frame.
  GaitPatternBase& gaitPattern_;
  size_t lastCycleToGoalCounter_;

  const size_t nStepsPerCycle_;
  std::vector<Step> previousPlan_;
};

} /* namespace locomotion_planner */
