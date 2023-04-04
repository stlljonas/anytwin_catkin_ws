/*
 * DynamicGaitsControllerRos.hpp
 *
 *  Created on: Feb 10, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// anymal_ctrl_dynamic_gaits
#include "anymal_ctrl_dynamic_gaits/DynamicGaitsController.hpp"

// ros
#include <anymal_msgs/SwitchController.h>
#include <anymal_msgs/GetAvailableControllers.h>
#include <anymal_msgs/GetActiveController.h>
#include <anymal_msgs/SetBoolean.h>
#include <any_msgs/SetUInt32.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <kindr_msgs/VectorAtPosition.h>
#include <motion_generation_msgs/GetAvailableGaits.h>
#include <motion_generation_msgs/SwitchGait.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <any_msgs/SetFloat32.h>
#include <motion_generation_msgs/SetStride.h>
#include <motion_generation_msgs/GetStride.h>

// loco ros
#include "loco_ros_anymal/visualization/LocomotionControllerRos.hpp"
#include "loco_ros_anymal/visualization/GaitPatterns.hpp"

// motion_generation_ros
#include "motion_generation_ros/SupportPolygonSequence.hpp"
#include "motion_generation_ros/MotionPlanVisualizer.hpp"
#include "motion_generation_ros/FootholdPlanVisualizer.hpp"
#include "motion_generation_ros/FinalComBox.hpp"
#include "motion_generation_ros/SwingTrajectoryGeneratorSplineOptimized.hpp"
#include "motion_generation_ros/SwingTrajectoryGeneratorSplineOptVisualizer.hpp"
#include "motion_generation_ros/StateCheckerPublisher.hpp"

// loco.
#include "loco/foothold_generation/FootholdPlanInvPend.hpp"

// terrains sensing.
#include "terrain_sensing/TerrainSensingRos.hpp"

// worker interface
#include <robot_control/workers/WorkerEvent.hpp>
#include <robot_control/workers/WorkerHandle.hpp>

// boost
#include <boost/thread.hpp>

namespace anymal_ctrl_dynamic_gaits_ros {

class DynamicGaitsControllerRos : public anymal_ctrl_dynamic_gaits::DynamicGaitsController,
                                  public loco_ros_anymal::LocomotionControllerRos
{
 private:
  using BaseRos = loco_ros_anymal::LocomotionControllerRos;
  using BaseController = anymal_ctrl_dynamic_gaits::DynamicGaitsController;

 public:
  DynamicGaitsControllerRos();
  ~DynamicGaitsControllerRos() override = default;

  bool create() override;
  bool advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) override;
  bool initialize() override;

  bool preStop() override;
  bool stop() override;

  anymal_motion_control::SwitchResult goToReferenceType(anymal_motion_control::ReferenceType referenceType) override;
  void goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) override;

 protected:
  void initRos() override;
  void updateRos();
  void updateMaximumPoseAndTwist(const std::string& gaitName);
  void shutdownRos() override;
  void publishRos() override;

  bool publishWorker(const robot_control::WorkerEvent& event);

  bool goToModeCallback(
      anymal_msgs::SwitchController::Request &req,
      anymal_msgs::SwitchController::Response &res);

  bool getAvailableModesCallback(
      anymal_msgs::GetAvailableControllers::Request &req,
      anymal_msgs::GetAvailableControllers::Response &res) const;

  bool getActiveModeCallback(
      anymal_msgs::GetActiveController::Request &req,
      anymal_msgs::GetActiveController::Response &res) const;

  bool getAvailableGaitsCallback(
      motion_generation_msgs::GetAvailableGaits::Request &req,
      motion_generation_msgs::GetAvailableGaits::Response &res);

  bool switchGaitCallback(
      motion_generation_msgs::SwitchGait::Request &req,
      motion_generation_msgs::SwitchGait::Response &res);

  bool switchDefaultConfigCallback(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);

  bool setStrideDurationCallback(
      motion_generation_msgs::SetStride::Request &req,
      motion_generation_msgs::SetStride::Response &res);

  bool getStrideDurationCallback(
      motion_generation_msgs::GetStride::Request &req,
      motion_generation_msgs::GetStride::Response &res);

  bool setBaseHeightAboveGroundCallback(
      any_msgs::SetFloat32::Request& req,
      any_msgs::SetFloat32::Response& res);

  //! Helper function: Convert contact schedule to anymal msgs response.
  void contactScheduleStatusToResponse(
      loco::contact_schedule::ContactScheduleSwitchStatus status,
      anymal_msgs::SwitchController::Response &res) const;

  //! Helper function: Convert contact schedule to mostion_generation msgs response.
  void contactScheduleStatusToResponse(
      loco::contact_schedule::ContactScheduleSwitchStatus status,
      motion_generation_msgs::SwitchGait::Response &res) const;

  //! Switch to gait with name gaitName and update max pose twist for that gait.
  loco::contact_schedule::ContactScheduleSwitchStatus updateGaitAndPoseTwist(const std::string& gaitName, bool updateContactSchedule = true);

  // Service calls.
  ros::ServiceServer getAvailableGatisService_;
  ros::ServiceServer switchGaitService_;
  ros::ServiceServer switchToDefaultConfigService_;
  ros::ServiceServer setStrideDurationService_;
  ros::ServiceServer getStrideDurationService_;
  ros::ServiceServer setBaseHeightAboveGround_;

  // Handle for publishing worker.
  robot_control::WorkerHandle publishWorkerHandle_;

  //! Mutex for locomotion controller.
  mutable boost::shared_mutex mutexLocomotionController_;

  //! A module which visualizes the contact schedule.
  loco_ros_anymal::GaitPatterns gaitPatternVisualizer_;

  //! Publishers.
  ros::Publisher minTwistPublisher_;
  ros::Publisher maxTwistPublisher_;
  ros::Publisher minPosePublisher_;
  ros::Publisher maxPosePublisher_;

  //! A module which visualizes a sequence of support polygons.
  SupportPolygonSequence supportPolygonSequence_;

  //! A module which visualizes the motion plan.
  MotionPlanVisualizer motionPlanVisualizer_;

  //! A module which visualizes the foothold plan.
  FootholdPlanVisualizer footholdPlanVisualizer_;

  //! A copy of the motion plan computed by the locomotion modules.
  zmp::MotionPlan motionPlan_;

  //! Foothold plan consisting of footholds and constraint polygons.
  loco::foothold_generator::FootholdPlanInvPend footholdPlan_;

  //! Motion plan for end effectors.
  std::vector<sto::MotionPlan> swingPlan_;

  //! A module which visualizes the final box constraints.
  FinalComBox finalComBox_;

  //! Visualization of swing leg trajectory.
  anymal_ctrl_dynamic_gaits_ros::SwingTrajectoryGeneratorSplineOptimized swingTrajectoryOptimizedVisualizer_;
  anymal_ctrl_dynamic_gaits_ros::SwingTrajectoryGeneratorSplineOptVisualizer swingTrajectorySplineOptVisualizer_;

  //! Publishing stability indications.
  anymal_ctrl_dynamic_gaits_ros::StateCheckerPublisher stateCheckerPublisher_;

  //! color codes for motion plan.
  std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>> colors_;
  unsigned int colorId_;

  //! Helper variable.
  bool updateColors_;

  //! Terrain sensing Ros object.
  std::unique_ptr<TerrainSensingRos> terrainSensingRos_;
};

} /* namespace anymal_ctrl_dynamic_gaits_ros */
