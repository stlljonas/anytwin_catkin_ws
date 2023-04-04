/*
 * DynamicGaitsControllerRos.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: C. Dario Bellicoso
 */

// loco crawling ros
#include "anymal_ctrl_dynamic_gaits_ros/DynamicGaitsControllerRos.hpp"
#include "anymal_ctrl_dynamic_gaits_ros/topology_conversions.hpp"

// message_logger
#include <message_logger/message_logger.hpp>

// robot_control
#include <robot_control/workers/WorkerOptions.hpp>

// ros
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

// loco
#include "loco/foot_placement_strategy/FootPlacementStrategyOptimized.hpp"

// std
#include <chrono>
#include <thread>
#include <condition_variable>


namespace anymal_ctrl_dynamic_gaits_ros {

DynamicGaitsControllerRos::DynamicGaitsControllerRos()
    : DynamicGaitsController("dynamic_gaits_ros"),
      BaseRos(),
      supportPolygonSequence_(),
      motionPlanVisualizer_(),
      footholdPlanVisualizer_(),
      motionPlan_(),
      footholdPlan_(),
      swingPlan_(),
      finalComBox_(),
      swingTrajectoryOptimizedVisualizer_(),
      stateCheckerPublisher_(),
      colors_(),
      colorId_(0u),
      updateColors_(false),
      terrainSensingRos_(nullptr)
{
  // Color table.
  colors_.reserve(18);
  colors_.emplace_back(loco::Vector(230.0/255, 25.0/255,  75.0/255));    // Red
  colors_.emplace_back(loco::Vector(60.0/255,  180.0/255, 75.0/255));    // Green
  colors_.emplace_back(loco::Vector(255.0/255, 225.0/255, 25.0/255));    // Yellow
  colors_.emplace_back(loco::Vector(0.0/255,   130.0/255, 200.0/255));   // Blue
  colors_.emplace_back(loco::Vector(245.0/255, 130.0/255, 48.0/255));    // Orange
  colors_.emplace_back(loco::Vector(145.0/255, 30.0/255,  180.0/255));   // Purple
  colors_.emplace_back(loco::Vector(70.0/255,  240.0/255, 240.0/255));   // Cyan
  colors_.emplace_back(loco::Vector(240.0/255, 50.0/255,  230.0/255));   // Maganta
  colors_.emplace_back(loco::Vector(210.0/255, 245.0/255, 60.0/255));    // Lime
  colors_.emplace_back(loco::Vector(0.0/255,   128.0/255, 128.0/255));   // Teal
  colors_.emplace_back(loco::Vector(230.0/255, 190.0/255, 255.0/255));   // Lavender
  colors_.emplace_back(loco::Vector(170.0/255, 110.0/255, 40.0/255));    // Brow
  colors_.emplace_back(loco::Vector(255.0/255, 250.0/255, 200.0/255));   // Beige
  colors_.emplace_back(loco::Vector(128.0/255, 0.0/255,   0.0/255));     // Maroon
  colors_.emplace_back(loco::Vector(170.0/255, 255.0/255, 195.0/255));   // Mint
  colors_.emplace_back(loco::Vector(128.0/255, 128.0/255, 0.0/255));     // Olive
  colors_.emplace_back(loco::Vector(255.0/255, 215.0/255, 180.0/255));   // Coral
  colors_.emplace_back(loco::Vector(0.0/255,   0.0/255,   128.0/255));   // Navy

  // Set color vectors.
  supportPolygonSequence_.setColorVector(colors_);
  motionPlanVisualizer_.setColorVector(colors_);
  footholdPlanVisualizer_.setColorVector(colors_);
  swingTrajectoryOptimizedVisualizer_.setColorVector(colors_);
}

bool DynamicGaitsControllerRos::create() {
  BaseRos::setNodeHandle(ros::NodeHandle("~"));

  if(!DynamicGaitsController::create()) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::create] Failed to create DynamicGaitsController objects.");
    return false;
  }

  anymal_msgs::GetAvailableControllers availableControllers;
  getAvailableModesCallback(availableControllers.request, availableControllers.response);
  setAvailableOperationModesForReferenceType(anymal_motion_control::ReferenceType::POSE, {"stand"});
  setAvailableOperationModesForReferenceType(anymal_motion_control::ReferenceType::TWIST, availableControllers.response.available_controllers);

  return true;
}

bool DynamicGaitsControllerRos::initialize() {
  if(!DynamicGaitsController::initialize()) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::initialize] Failed to initialize DynamicGaitsController.");
    return false;
  }

  initRos();
  updateColors_ = false;
  colorId_ = 0u;

  //--- start logging in a worker thread
  robot_control::WorkerOptions publishWorkerOptions;
  publishWorkerOptions.autostart_ = false;
  publishWorkerOptions.frequency_ = 25.0;
  publishWorkerOptions.name_ = this->getName() + "_publisher";
  publishWorkerOptions.priority_ = 0;
  publishWorkerOptions.callback_ = boost::bind(&DynamicGaitsControllerRos::publishWorker, this, _1);
  publishWorkerHandle_ = getWorkerManager().addWorker(publishWorkerOptions);
  getWorkerManager().startWorker(publishWorkerHandle_);
  //---

  //--- publish the maximal and minimal twists and poses
  updateGaitAndPoseTwist(getContactSchedule().getActiveGaitName(), false);
  //---

  return true;
}

bool DynamicGaitsControllerRos::advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) {
  bool success = true;
  {
    boost::unique_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    success &= DynamicGaitsController::advance(command, commandMutex);

    // Advance colors. Note: Call after advancing DynamicGaitsController!!
    if (!updateColors_) {
      try {
        if (static_cast<loco::ComSupportControlZmp*>(getTorsoControlPtr()->getComSupportControlPtr())->isFirstSupportPolygonNew()) {
          colorId_ = robot_utils::intmod(colorId_+1, colors_.size());
          updateColors_ = true;
        }
      } catch (...) { }
    }
  }
  return success;
}

bool DynamicGaitsControllerRos::preStop() {

  // Send stop signal to mission controller if controller still running
  if (isRunning()) {
    {
      boost::unique_lock<boost::shared_mutex> lock(mutexLocomotionController_);
      missionController_->setTargetLocomotionMode(loco::MissionControlZmp::LocomotionMode::ModeStopping);
    }
    // Wait for 250 ms for locomotion mode to be changed
    const std::chrono::duration<int, std::milli> timeout(250);
    if (!missionController_->waitForLocomotionModeSwitch(loco::MissionControlZmp::LocomotionMode::ModeStopping, timeout)) {
      MELO_WARN_STREAM("[DynamicGaitsControllerRos::preStop] Switch to stop timeout!");
    }
    // Wait for 5 s for robot to reach rest
    const std::chrono::duration<int, std::milli> period(10);
    int timeout_it = 500;
    while (!missionController_->isStopped() && isRunning()) {
      std::this_thread::sleep_for(period);
      timeout_it--;
      if (timeout_it <= 0) {
        MELO_ERROR("[DynamicGaitsControllerRos::preStop] Safe stop timeout!");
        break;
      }
    }
  }  

  if (!BaseController::preStop()) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::preStop] Failed to pre-stop base controller.");
    return false;
  }

  if (!getWorkerManager().cancelWorker(publishWorkerHandle_, true)) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::preStop] Failed to cancel worker.");
    return false;
  }

  shutdownRos();

  return true;
}

bool DynamicGaitsControllerRos::stop() {
  MELO_INFO("[DynamicGaitsControllerRos] Stopping controller.");
  return BaseController::stop();
}

void DynamicGaitsControllerRos::initRos() {
  shutdownRos();

  getAvailableGatisService_ = nodeHandle_.advertiseService("/" + this->getName() + "/get_available_gaits", &DynamicGaitsControllerRos::getAvailableGaitsCallback, this);
  switchGaitService_ = nodeHandle_.advertiseService("/" + this->getName() + "/switch_gait", &DynamicGaitsControllerRos::switchGaitCallback, this);
  switchToDefaultConfigService_ = nodeHandle_.advertiseService("/" + this->getName() + "/switch_to_default_config", &DynamicGaitsControllerRos::switchDefaultConfigCallback, this);

  setStrideDurationService_ = nodeHandle_.advertiseService("/" + this->getName() + "/set_stride_duration", &DynamicGaitsControllerRos::setStrideDurationCallback, this);
  getStrideDurationService_ = nodeHandle_.advertiseService("/" + this->getName() + "/get_stride_duration", &DynamicGaitsControllerRos::getStrideDurationCallback, this);

  minTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("/commands/twist_min", 10, true);
  maxTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("/commands/twist_max", 10, true);
  minPosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/commands/pose_min", 10, true);
  maxPosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/commands/pose_max", 10, true);

  gaitPatternVisualizer_.initialize(nodeHandle_, "/loco_ros/gait_patterns");
  supportPolygonSequence_.initialize(nodeHandle_, "/dynamic_gaits_ros/support_polygons");
  motionPlanVisualizer_.initialize(nodeHandle_, "/dynamic_gaits_ros/");
  footholdPlanVisualizer_.initialize(nodeHandle_, "/dynamic_gaits_ros/foothold_plan");
  finalComBox_.initialize(nodeHandle_, "/dynamic_gaits_ros/final_com_box");
  swingTrajectoryOptimizedVisualizer_.initialize(nodeHandle_, "/dynamic_gaits_ros/swing_trajectory");
  swingTrajectorySplineOptVisualizer_.initialize(nodeHandle_, "/dynamic_gaits_ros/swing_trajectory_opt");
  stateCheckerPublisher_.initialize(nodeHandle_, "/" + this->getName() + "/stability_indicator_value");

  setBaseHeightAboveGround_ = nodeHandle_.advertiseService("/" + this->getName() + "/set_base_height", &DynamicGaitsControllerRos::setBaseHeightAboveGroundCallback, this);

  BaseRos::initRos();
}

void DynamicGaitsControllerRos::shutdownRos() {
  minTwistPublisher_.shutdown();
  maxTwistPublisher_.shutdown();
  minPosePublisher_.shutdown();
  maxPosePublisher_.shutdown();

  getAvailableGatisService_.shutdown();
  switchGaitService_.shutdown();
  switchToDefaultConfigService_.shutdown();
  setStrideDurationService_.shutdown();
  getStrideDurationService_.shutdown();

  setBaseHeightAboveGround_.shutdown();

  gaitPatternVisualizer_.shutdown();
  supportPolygonSequence_.shutdown();
  motionPlanVisualizer_.shutdown();
  footholdPlanVisualizer_.shutdown();
  finalComBox_.shutdown();
  swingTrajectoryOptimizedVisualizer_.shutdown();
  swingTrajectorySplineOptVisualizer_.shutdown();
  stateCheckerPublisher_.shutdown();

  BaseRos::shutdownRos();
}

void DynamicGaitsControllerRos::publishRos() {
  gaitPatternVisualizer_.publish();
  supportPolygonSequence_.publish();
  motionPlanVisualizer_.publish();
  footholdPlanVisualizer_.publish();
  finalComBox_.publish();

  if (DynamicGaitsController::usingOptimizedSwingTrajectory()) {
    swingTrajectoryOptimizedVisualizer_.publish();
  } else {
    swingTrajectorySplineOptVisualizer_.publish();
  }

  if (useStateChecker_) {
    stateCheckerPublisher_.publish();
  }

  BaseRos::publishRos();
}

bool DynamicGaitsControllerRos::publishWorker(const robot_control::WorkerEvent& event) {
  updateRos();
  publishRos();
  return true;
}

bool DynamicGaitsControllerRos::getAvailableModesCallback(anymal_msgs::GetAvailableControllers::Request &req,
                                                anymal_msgs::GetAvailableControllers::Response &res) const {
  res.available_controllers.clear();
  res.available_controllers.reserve(static_cast<unsigned int>(ControllerMode::SIZE));

  // Read available gait
  try {
    boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
      for (const auto& gait : getContactSchedule().getMapGaitNameToId()) {
        if (getContactSchedule().isValidGait(gait.first)) {
          res.available_controllers.push_back(gait.first);
        }
      }
  } catch (...) { return false; }

  return true;
}

bool DynamicGaitsControllerRos::getActiveModeCallback(
    anymal_msgs::GetActiveController::Request &req,
    anymal_msgs::GetActiveController::Response &res) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
  res.active_controller_locomotion_mode = missionController_->getLocomotionModeName();
  return true;
}

bool DynamicGaitsControllerRos::goToModeCallback(anymal_msgs::SwitchController::Request &req,
                                       anymal_msgs::SwitchController::Response &res) {

  auto status = loco::contact_schedule::ContactScheduleSwitchStatus::Error;
  const std::string& name = req.name;
  bool isValidGait;

  {
    boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    isValidGait = getContactSchedule().isValidGait(name);
  }

  // Check if requested mode is a valid gait.
  if (isValidGait) {
    MELO_INFO("[DynamicGaitsControllerRos] Called service to switch gait.");
    status = updateGaitAndPoseTwist(name);
    contactScheduleStatusToResponse(status, res);
    return true;
  }

  // Check if requested mode is stand
  if (name == "stand") {
    res.status = motion_generation_msgs::SwitchGait::Response::STATUS_SWITCHED;
    return true;
  }

  MELO_WARN_FP("[DynamicGaitsControllerRos] Requested mode '%s' is invalid.", name.c_str());
  res.status = motion_generation_msgs::SwitchGait::Response::STATUS_NOTFOUND;
  return true;
}

bool DynamicGaitsControllerRos::getAvailableGaitsCallback(
    motion_generation_msgs::GetAvailableGaits::Request &req,
    motion_generation_msgs::GetAvailableGaits::Response &res) {
  try {
    boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    const auto& map = getContactSchedule().getMapGaitNameToId();

    res.available_gaits_names.clear();
    res.available_gaits_ids.clear();
    const auto gaitNamesSize = map.size();
    res.available_gaits_names.reserve(gaitNamesSize);
    res.available_gaits_ids.reserve(gaitNamesSize);
    {
      for (const auto& pair : map) {
        if (getContactSchedule().isValidGait(pair.first)) {
          res.available_gaits_names.push_back(pair.first);
          res.available_gaits_ids.push_back(pair.second);
        }
      }
    }
    return true;
  } catch (...) {
    return false;
  }
}

bool DynamicGaitsControllerRos::switchGaitCallback(
    motion_generation_msgs::SwitchGait::Request &req,
    motion_generation_msgs::SwitchGait::Response &res) {
  try {
    const auto status = updateGaitAndPoseTwist(req.name);
    contactScheduleStatusToResponse(status, res);
    return true;
  } catch (...) {
    return false;
  }
}

bool DynamicGaitsControllerRos::switchDefaultConfigCallback(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res) {
  try {
    boost::unique_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    loco::contact_schedule::ContactScheduleSwitchStatus status;
    getContactSchedulePtr()->executeOneCycle(status);
    return true;
  } catch (...) {
    return false;
  }
}

bool DynamicGaitsControllerRos::setStrideDurationCallback(
    motion_generation_msgs::SetStride::Request &req,
    motion_generation_msgs::SetStride::Response &res) {
  try {
    boost::unique_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    getContactSchedulePtr()->setActiveNominalStrideDuration(req.strideDuration);
    return true;
  } catch (...) {
    return false;
  }
}

bool DynamicGaitsControllerRos::getStrideDurationCallback(
    motion_generation_msgs::GetStride::Request &req,
    motion_generation_msgs::GetStride::Response &res) {
  try {
    boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    res.strideDuration = getContactSchedule().getStrideDuration();
    return true;
  } catch (...) {
    return false;
  }
}

bool DynamicGaitsControllerRos::setBaseHeightAboveGroundCallback(
    any_msgs::SetFloat32::Request& req,
    any_msgs::SetFloat32::Response& res) {
  auto* comControl = dynamic_cast<loco::ComSupportControlZmp*>(getTorsoControlPtr()->getComSupportControlPtr());
  if (comControl == nullptr) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::setBaseHeightAboveGroundCallback] Failed to cast com support control module.");
    res.success = false;
    return false;
  }
  comControl->setDesiredHeightAboveGroundOnFlatTerrain(req.data);
  res.success = true;

  return true;
}

void DynamicGaitsControllerRos::updateRos() {
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);

    BaseRos::updateRos(getWholeBody(), getTerrainModel(), getHeadingGenerator(), getMotionController(), getFootPlacementStrategy());

    // Update Gait Pattern Visualizer.
    const auto* gait = dynamic_cast<const loco::contact_schedule::ContactScheduleAnymalBase*>(&getContactSchedule());
    if (gait == nullptr) {
      MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to cast gait.");
    } else {
      gaitPatternVisualizer_.updateContactSchedule(gait, 1.5);
    }

    // Get motion plan.
    if (motionPlanVisualizer_.getNumSubscribers() > 0u) {
      const auto* comCtrl = dynamic_cast<const loco::ComSupportControlZmp*>(&getTorsoControl().getComSupportControl());
      if (comCtrl == nullptr) {
        MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to cast Com Support Control.");
      } else {
        comCtrl->getMotionPlan(motionPlan_);
      }
    }

    // Get foothold plan.
    if (footholdPlanVisualizer_.getNumSubscribers() > 0u) {
      switch(this->footholdGenerationOptimizer_) {
        case loco::foothold_generator::FootholdGenerationOptimizer::BlindQP : {
          const auto* fhGen = dynamic_cast<const loco::FootPlacementStrategyOptimized<loco::FootholdGeneratorOptimizedInvPend, loco::foothold_generator::FootholdPlanInvPend>*>(&getFootPlacementStrategy());
          if (fhGen == nullptr) {
            MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to cast foot-placement strategy.");
          } else {
            fhGen->getFootholdPlan(footholdPlan_);
          }
        } break;

        default : {
          MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Unhandled foothold generation technique!");
        }
      }
    }

    // Get swing trajectory plan.
    if (DynamicGaitsController::usingOptimizedSwingTrajectory() && swingTrajectoryOptimizedVisualizer_.getNumSubscribers() > 0u) {
      const auto* fhS = dynamic_cast<const loco::SwingTrajectoryGeneratorSplineOptimized*>(&getFootPlacementStrategy().getSwingTrajectoryGenerator());
      if (fhS == nullptr) {
        MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to cast swing-trajectory generator.");
      } else {
        fhS->getMotionPlan(swingPlan_);
      }
    }
  }

  // Update color index.
  if (updateColors_) {
    supportPolygonSequence_.setColorVectorId(colorId_);
    motionPlanVisualizer_.setColorVectorId(colorId_);
    updateColors_ = false;
  }

  // Update visualizers.
  if(!supportPolygonSequence_.update(motionPlan_)) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to update support polygon sequence!");
  }
  if(!finalComBox_.update(motionPlan_)) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to update final com box!");
  }
  if(!motionPlanVisualizer_.update(motionPlan_)) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to update motion plan!");
  }

  if(!footholdPlanVisualizer_.update(footholdPlan_)) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to update foothold plan!");
  }

  if(!swingTrajectoryOptimizedVisualizer_.update(swingPlan_)) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to update swing trajectory optimizer!");
  }

  if (!DynamicGaitsController::usingOptimizedSwingTrajectory()) {
    try {
      swingTrajectorySplineOptVisualizer_.update(dynamic_cast<const loco::SwingTrajectoryGeneratorSplineOpt&>(getFootPlacementStrategy().getSwingTrajectoryGenerator()));
    } catch (...) {
      MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to update swing trajectory!");
    }
  }

  if (useStateChecker_ && !stateCheckerPublisher_.update(stateChecker_->getIsSafe())) {
    MELO_WARN_STREAM("[DynamicGaitsControllerRos::updateRos] Failed to update state checker publisher!");
  }

}

void DynamicGaitsControllerRos::updateMaximumPoseAndTwist(const std::string& gaitName) {
  //--- publish the maximal and minimal twists and poses
  geometry_msgs::TwistStamped minTwistStampedMsg, maxTwistStampedMsg;
  geometry_msgs::PoseStamped minPoseStampedMsg, maxPoseStampedMsg;
  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 twistLinear, twistAngular;


  {
    ROS_INFO_STREAM("[DynamicGaitsControllerRos::updateMaximumPoseAndTwist] publish new minimal/maximal twist/pose");

    boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    kindr_ros::convertToRosGeometryMsg(
            missionController_->getMaximumBaseTwistInControlFrameOfGait(gaitName).getTranslationalVelocity(), twistLinear);
    maxTwistStampedMsg.twist.linear = twistLinear;
    maxTwistStampedMsg.twist.linear.z = 0.0;
    kindr_ros::convertToRosGeometryMsg(
            missionController_->getMaximumBaseTwistInControlFrameOfGait(gaitName).getRotationalVelocity(), twistAngular);
    maxTwistStampedMsg.twist.angular = twistAngular;
    maxTwistStampedMsg.twist.angular.x = 0.0;
    maxTwistStampedMsg.twist.angular.y = 0.0;
    minTwistStampedMsg.twist.linear.x = -maxTwistStampedMsg.twist.linear.x;
    minTwistStampedMsg.twist.linear.y = -maxTwistStampedMsg.twist.linear.y;
    minTwistStampedMsg.twist.angular.z = -maxTwistStampedMsg.twist.angular.z;

    kindr_ros::convertToRosGeometryMsg(missionController_->getMinimalPoseOffset(), pose);
    minPoseStampedMsg.pose = pose;
    kindr_ros::convertToRosGeometryMsg(missionController_->getMaximalPoseOffset(), pose);
    maxPoseStampedMsg.pose = pose;
  }

  const ros::Time rosTime = ros::Time::now();
  minTwistStampedMsg.header.stamp = rosTime;
  maxTwistStampedMsg.header.stamp = rosTime;
  minPoseStampedMsg.header.stamp = rosTime;
  maxPoseStampedMsg.header.stamp = rosTime;
  minTwistPublisher_.publish(minTwistStampedMsg);
  maxTwistPublisher_.publish(maxTwistStampedMsg);
  minPosePublisher_.publish(minPoseStampedMsg);
  maxPosePublisher_.publish(maxPoseStampedMsg);
  //---
}

void DynamicGaitsControllerRos::contactScheduleStatusToResponse(
    loco::contact_schedule::ContactScheduleSwitchStatus status,
    anymal_msgs::SwitchController::Response &res) const {
  switch(status) {
    case loco::contact_schedule::ContactScheduleSwitchStatus::Error: {
      res.status = anymal_msgs::SwitchController::Response::STATUS_NOTFOUND;
    } break;

    case loco::contact_schedule::ContactScheduleSwitchStatus::NotFound: {
      res.status = anymal_msgs::SwitchController::Response::STATUS_NOTFOUND;
    } break;

    case loco::contact_schedule::ContactScheduleSwitchStatus::Running: {
      res.status = anymal_msgs::SwitchController::Response::STATUS_SWITCHED;
    } break;

    case loco::contact_schedule::ContactScheduleSwitchStatus::Switched: {
      res.status = anymal_msgs::SwitchController::Response::STATUS_SWITCHED;
    } break;

    default : {
      res.status = anymal_msgs::SwitchController::Response::STATUS_ERROR;
    } break;
  }
}

loco::contact_schedule::ContactScheduleSwitchStatus DynamicGaitsControllerRos::updateGaitAndPoseTwist(const std::string& gaitName, bool updateContactSchedule) {
  unsigned int gaitId;
  loco::contact_schedule::ContactScheduleSwitchStatus status = loco::contact_schedule::ContactScheduleSwitchStatus::Undefined;

  // Get gait index.
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    gaitId = getContactSchedule().getMapGaitNameToId().at(gaitName);
  }

  // (1) Compute gait transition.
  if (updateContactSchedule) {
    boost::unique_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    getContactSchedulePtr()->setDesiredGaitById(gaitId, status);
  }

  // (2) Set pose twist for desired gait.
  updateMaximumPoseAndTwist(gaitName);

  return status;
}

anymal_motion_control::SwitchResult DynamicGaitsControllerRos::goToReferenceType(anymal_motion_control::ReferenceType referenceType) {
  // Check if requested mode is a valid operation mode
  switch (referenceType) {
    case (anymal_motion_control::ReferenceType::TWIST): {
      MELO_INFO("[DynamicGaitsControllerRos] Call service to start walking.");
      {
        MELO_INFO_STREAM(getContactSchedule().getActiveGaitName())
        updateGaitAndPoseTwist(getContactSchedule().getActiveGaitName(), false);
        boost::unique_lock<boost::shared_mutex> lock(mutexLocomotionController_);
        missionController_->setTargetLocomotionMode(loco::MissionControlZmp::LocomotionMode::ModeWalking);
      }

      // Wait for 250 ms for locomotion mode to be changed
      const std::chrono::duration<int, std::milli> timeout(250);
      if (!missionController_->waitForLocomotionModeSwitch(loco::MissionControlZmp::LocomotionMode::ModeWalking, timeout)) {
        MELO_WARN_STREAM("[DynamicGaitsControllerRos] Locomotion mode switch timeout!");
        return anymal_motion_control::SwitchResult::ERROR;
      }
    } break;

    case (anymal_motion_control::ReferenceType::POSE): {
      MELO_INFO("[DynamicGaitsControllerRos] Call service to transit to stand.");
      {
        boost::unique_lock<boost::shared_mutex> lock(mutexLocomotionController_);
        missionController_->setTargetLocomotionMode(loco::MissionControlZmp::LocomotionMode::ModeStanding);
      }

      // Wait for 5 seconds for locomotion mode to be changed
      const std::chrono::duration<int, std::milli> timeout(5000);
      if (!missionController_->waitForLocomotionModeSwitch(loco::MissionControlZmp::LocomotionMode::ModeStanding, timeout)) {
        MELO_WARN_STREAM("[DynamicGaitsControllerRos] Locomotion mode switch timeout!");
        return anymal_motion_control::SwitchResult::ERROR;
      }
    } break;

    case (anymal_motion_control::ReferenceType::ACTION):
      MELO_WARN("[DynamicGaitsControllerRos] Action reference type is not implemented.");
      return anymal_motion_control::SwitchResult::ERROR;

    case (anymal_motion_control::ReferenceType::NA):
      MELO_WARN("[DynamicGaitsControllerRos] Reference type is not available.");
      return anymal_motion_control::SwitchResult::ERROR;
  }

  return anymal_motion_control::SwitchResult::SWITCHED;
}

void DynamicGaitsControllerRos::goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) {

  // call mode switch
  anymal_msgs::SwitchController switchController;
  switchController.request.name = operationMode;
  switchController.request.block = static_cast<unsigned int>(true);
  goToModeCallback(switchController.request, switchController.response);

  // process return value
  auto result = anymal_motion_control::SwitchResult::ERROR;
  switch(switchController.response.status) {
    case anymal_msgs::SwitchController::Response::STATUS_SWITCHED:
    case anymal_msgs::SwitchController::Response::STATUS_RUNNING:
      result = anymal_motion_control::SwitchResult::SWITCHED;
      action->setSucceeded(result, "Successfully switched operation mode " + operationMode);
      return;
    case anymal_msgs::SwitchController::Response::STATUS_NOTFOUND:
      result = anymal_motion_control::SwitchResult::NOT_FOUND;
      action->setAborted(result, "Operation mode "  + operationMode + " not found");
      return;
    case anymal_msgs::SwitchController::Response::STATUS_ERROR:
      result = anymal_motion_control::SwitchResult::ERROR;
      action->setAborted(result, "Could not switch to operation mode " + operationMode);
      return;
  }
  action->setAborted(result, "Could not switch operation mode");
}

} /* namespace anymal_ctrl_dynamic_gaits_ros */
