/**
 * @authors     Peter Fankhauser, Francisco Giraldez Gamez, Valentin Yuryev
 * @affiliation ANYbotics
 * @brief       Declaration of LocomotionPlanner class
 */

#pragma once

#include "locomotion_planner/common/Parameters.hpp"
#include "locomotion_planner/common/ElevationMapUser.hpp"
#include "locomotion_planner/common/PlanningData.hpp"
#include "locomotion_planner/footstep_planner/FootstepPlannerBase.hpp"
#include "locomotion_planner/gait_pattern/GaitPatternCrawling.hpp"
#include "locomotion_planner/foothold_optimizer/FootholdOptimizerElevationMapAndKinematics.hpp"
#include "locomotion_planner/swing_trajectory_planner/SwingTrajectoryPlannerBase.hpp"

#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_ros/FreeGaitActionClient.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include "common/type_defs.hpp"
#include "foothold_optimizer/FootholdOptimizerSameHeight.hpp"

namespace locomotion_planner {

class LocomotionPlanner
{
 public:
  enum class GoalPoseStatus {
    ALREADY_THERE = 0,
    IGNORED_SAME_AS_CURRENT = 1,
    OVERWRITE_CURRENT_GOAL = 2,
    ACCEPTED_NEW_GOAL = 3,
    REJECTED_FRAME_ERROR = 4 ,
    REJECTED_INVALID_TRANSFORM = 5
  };

  enum class GoalPoseResult {
    SUCCESS = 1,
    PREEMPTED = 2,
    PLANNER_FAILURE = 3,
    EXECUTION_FAILURE = 4
  };

  LocomotionPlanner(free_gait::AdapterBase& adapter, Parameters& parameters);
  virtual ~LocomotionPlanner() = default;

  void registerResultCallback(std::function<void(const LocomotionPlanner::GoalPoseResult&)> callback);
  void registerSendGoalCallback(std::function<void(const free_gait::StepQueue&)> callback);
  void registerGetActionStateCallback(
      std::function<bool(free_gait::FreeGaitActionClient::ActionState&, free_gait::ExecutorState&)> callback);
  void registerGetTransformCallback(std::function<bool(const std::string&, const std::string&, Transform&)> callback);
  void registerVisualizePlanningDataCallback(std::function<void(const PlanningData&)> callback);

  LocomotionPlanner::GoalPoseStatus setNewGoalPose(const std::string &frameId,
                                                   const Pose &pose);
  LocomotionPlanner::GoalPoseStatus setNewRelativeGoalPose(const std::string &frameId,
                                                   const Pose &pose);
  void setNewTwistCommand(const Twist& twist);
  void getElevationMapRegion(grid_map::Position& position, grid_map::Length& length);
  void setElevationMap(const grid_map::GridMap& elevationMap);
  void clearElevationMap();

  void initialize();
  void start();
  void stop();

 private:
  LocomotionPlanner::GoalPoseStatus setNewGoalPose(const Pose& pose, const double speedFactor);
  Pose getTargetPoseFromRelativeGoalPose(const Pose& relativePoseInBaseFrame, const std::string& goalFrameId) const;
  bool compareGoalPoses(const Pose& newGoalPose, const Pose& oldGoalPose, const double tolerance) const;
  void defineNewGoalPoseFromTwistCommand();
  void resetGoal();
  void spinPlanner();
  void plan();
  void sendPlan();

  //! @return the number of free gait action steps (not literal steps!) that are executed
  //! before new plan (replanning) is used.
  size_t getNumberOfStepsForReplanningHorizon() const;

  //! @return the expected upper planning time (might be exceeded in special circumstances).
  double getExpectedUpperPlanningTime() const;

 private:
  std::string newGoalFrameId_;
  std::string goalFrameId_;
  Pose goalPose_;
  Twist twistCommand_;
  PlanarTwist maxTwistCommand_;
  double speedFactor_;
  Parameters& parameters_;
  std::function<void(const free_gait::StepQueue&)> sendGoalCallback_;
  std::function<void(const GoalPoseResult&)> resultCallback_;
  std::function<bool(free_gait::FreeGaitActionClient::ActionState&, free_gait::ExecutorState&)> getActionStateCallback_;
  std::function<bool(const std::string&, const std::string&, Transform&)> getTransformCallback_;
  std::function<void(const PlanningData&)> visualizePlanningDataCallback_;
  free_gait::AdapterBase& adapter_;
  free_gait::StepQueue previousPlan_;
  free_gait::StepQueue plan_;
  PlanningData planningData_;
  std::thread spinnerThread_;
  std::thread plannerThread_;
  std::atomic<bool> planSuccessful_{false};
  std::atomic<bool> isNewGoal_{false};
  std::atomic<bool> isThereGoal_{false};
  std::atomic<bool> isTwistCommandActive_{false};
  std::atomic<bool> hasStartedPlanning_{false};
  std::atomic<bool> hasFinishedPlanning_{false};
  std::atomic<bool> isPlanSent_{false};
  std::atomic<bool> isLastPlanToGoal_{false};
  std::atomic<bool> isRequestedToStop_{false};
  PlannerModuleBase* currentPlannerModule_;
  std::unique_ptr<FootstepPlannerBase> footstepPlanner_;
  std::unique_ptr<GaitPatternBase> gaitPattern_;
  std::unique_ptr<FootholdOptimizerSameHeight> footholdOptimizerBackup_;
  std::unique_ptr<FootholdOptimizerElevationMapAndKinematics> footholdOptimizerElevationMapAndKinematics_;
  std::unique_ptr<SwingTrajectoryPlannerBase> swingTrajectoryPlanner_;
  std::unique_ptr<SwingTrajectoryPlannerBase> swingTrajectoryPlannerBackup_;
  std::shared_ptr<ElevationMapUser> elevationMapUser_;
  std::vector<std::tuple<LimbEnum, Position>> nominalFootholds_;
  std::vector<std::tuple<LimbEnum, Position>> optimizedFootholds_;
};

} /* namespace */
