/**
 * @authors     Peter Fankhauser, Francisco Giraldez Gamez, Valentin Yuryev
 * @affiliation ANYbotics
 * @brief       Declaration of LocomotionPlannerRos class
 */

#pragma once

#include "locomotion_planner/LocomotionPlanner.hpp"
#include "locomotion_planner/common/type_defs.hpp"
#include "locomotion_planner/common/Parameters.hpp"
#include "locomotion_planner/common/PlanningData.hpp"
#include <locomotion_planner_msgs/NavigateToGoalPoseAction.h>

#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_ros/FreeGaitActionClient.hpp>
#include <free_gait_ros/StepFrameConverter.hpp>
#include <free_gait_ros/StepRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <filters/filter_chain.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <thread>
#include <atomic>

namespace locomotion_planner {

class LocomotionPlannerRos
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param adapter the Free Gait adapter.
   */
  LocomotionPlannerRos(ros::NodeHandle& nodeHandle, free_gait::AdapterBase& adapter);

  /*!
   * Destructor.
   */
  virtual ~LocomotionPlannerRos() = default;

  bool initialize();
  void shutdown();

  void setInputToTwist();
  void setInputToPose();

  void setConfiguration(const std::string& configuration);
  std::string getConfiguration();

  bool isActive();

  void twistCommandCallback(const geometry_msgs::TwistStamped& message);
  PlanarTwist getMaxTwistCommandFast();
  PlanarTwist getMaxTwistCommandSlow();

 private:
  enum class LocomotionPlannerInputType {
    UNDEFINED = 0,
    TWIST = 1,
    POSE = 2
  };

  bool readParameters();
  void resultCallback(const LocomotionPlanner::GoalPoseResult& result);
  void sendGoalCallback(const free_gait::StepQueue& goal);
  bool getActionStateCallback(free_gait::FreeGaitActionClient::ActionState& actionState, free_gait::ExecutorState& executorState);
  bool getTransform(const std::string& sourceFrameId, const std::string& targetFrameId, Transform& transform);
  void updateElevationMap(const ros::TimerEvent& timerEvent);
  void visualizePlanningData(const PlanningData& planningData);

  void executeStepsActiveCallback();
  void executeStepsFeedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback);
  void executeStepsDoneCallback(const actionlib::SimpleClientGoalState& state, const free_gait_msgs::ExecuteStepsResult& result);

  void goalPoseGoalCallback();
  void goalPosePreemptCallback();

  bool waitForFootholdPlan();
  bool waitForActionCompletion();
  void executeLocomotionPlanner();
  bool stopLocomotionPlannerExecution();

  void runElevationMapUpdateThread();
  void shutdownPerception();

  void publishTwistLimits();

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Goal/command topics.
  ros::Subscriber twistCommandSubscriber_;
  std::string twistCommandTopic_;
  Pose goalToWorldFrameTransform_;

  //! Twist limits publisher
  ros::Publisher minTwistPublisher_;
  ros::Publisher maxTwistPublisher_;

  //! Goal pose action server.
  actionlib::SimpleActionServer<locomotion_planner_msgs::NavigateToGoalPoseAction> goalPoseServer_;
  locomotion_planner_msgs::NavigateToGoalPoseResult result_;

  //! Enable/disable percepton
  bool runPerception_;

  //! Elevation map.
  ros::ServiceClient getElevationMapService_;
  std::string elevationMapServiceName_;
  std::thread elevationMapUpdateThread_;
  ros::CallbackQueue elevationMapUpdateQueue_;
  ros::Timer elevationMapUpdateTimer_;
  ros::Duration elevationMapUpdateDuration_;
  ros::Duration maxAgeOfLastElevationMapUpdate_;
  ros::Time lastSuccessfulElevationMapUpdate_;
  filters::FilterChain<grid_map::GridMap> elevationMapFilterChain_;
  ros::Publisher elevationMapPublisher_;
  std::atomic<bool> updateElevationMap_{false};

  //! Action client.
  free_gait::FreeGaitActionClient actionClient_;

  //! Current state of the executor.
  free_gait::ExecutorState executorState_;

  //! Free Gait tools.
  free_gait::AdapterBase& adapter_;
  free_gait::StepRosConverter rosConverter_;
  free_gait::StepFrameConverter frameConverter_;

  //! Plan data visualizations.
  ros::Publisher footstepVisualizationPublisher_;

  //! Common parameters.
  Parameters parameters_;

  //! Locomotion planner (needs to come after adapter!).
  LocomotionPlanner locomotionPlanner_;

  //! Locomotion planner configuration
  std::string configuration_ = "default";

  //! Type of input taken
  LocomotionPlannerInputType inputType_ = LocomotionPlannerInputType::UNDEFINED;

  //! Flag to show if the LocomotionPlannerRos is active
  std::atomic<bool> isActive_{false};

};

} /* namespace */
