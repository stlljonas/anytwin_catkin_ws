/**
 * @authors     Peter Fankhauser, Gabriel Hottiger, Francisco Giraldez Gamez, Valentin Yuryev
 * @affiliation ANYbotics
 * @brief       Declaration and implementation of basic Free Gait ROS controller
 */

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <any_msgs/GetStrings.h>
#include <any_msgs/Toggle.h>
#include <anymal_msgs/GetAvailableControllers.h>
#include <anymal_msgs/SetBoolean.h>
#include <anymal_msgs/SwitchController.h>
#include <free_gait_msgs/ExecuteActionAction.h>
#include <free_gait_msgs/GetActions.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <anymal_ctrl_free_gait/FreeGait.hpp>
#include <anymal_motion_control/State.hpp>
#include <anymal_motion_control/typedefs.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <free_gait_anymal_common/AdapterAnymal.hpp>
#include <free_gait_anymal_common/AdapterAnymalPreview.hpp>
#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_ros/free_gait_ros.hpp>
#include <geometry_utils_ros/TransformListenerRos.hpp>
#include <loco_ros_anymal/visualization/LocomotionControllerRos.hpp>
#include <locomotion_planner/LocomotionPlannerRos.hpp>
#include <memory>
#include <message_logger/message_logger.hpp>
#include <param_io/get_param.hpp>
#include <param_io/set_param.hpp>
#include <robot_control/workers/workers.hpp>
#include <std_utils/timers/ChronoTimer.hpp>

namespace anymal_ctrl_free_gait {

template <class Task>
class FreeGaitRos : public Task, public loco_ros_anymal::LocomotionControllerRos {
 public:
  using BaseRos = loco_ros_anymal::LocomotionControllerRos;

  FreeGaitRos() : FreeGaitRos("anymal_ctrl_free_gait_ros") {
    dt_ = 0.0;  // Only reason for initializing this here: fix clang warning.
  }

  explicit FreeGaitRos(const std::string& controllerName) : Task(controllerName), dt_(0.0) {}

  ~FreeGaitRos() override = default;

  bool create() override {
    BaseRos::setNodeHandle(nodeHandle_);

    if (!FreeGait::create()) {
      return false;
    }
    if (!stepActionServer_) {
      stepActionServer_ = std::unique_ptr<free_gait::FreeGaitActionServer>(
          new free_gait::FreeGaitActionServer(nodeHandle_, "/anymal_ctrl_free_gait/execute_steps", *(this->executor_), *(this->adapter_)));
      stepActionServer_->initialize();
    }

    locomotionPlanner_ = std::unique_ptr<locomotion_planner::LocomotionPlannerRos>(
        new locomotion_planner::LocomotionPlannerRos(nodeHandle_, *(this->adapter_)));

    actionLoaderClient_.reset(
        new actionlib::SimpleActionClient<free_gait_msgs::ExecuteActionAction>("/free_gait_action_loader/execute_action", true));

    getAvailableActions_ = nodeHandle_.serviceClient<free_gait_msgs::GetActions>("/free_gait_action_loader/list_actions");
    getDirectoryContentsService_ =
        nodeHandle_.advertiseService(this->getName() + "/get_directory_contents", &FreeGaitRos::getDirectoryContentsService, this);

    try {
      auto tfListenerPtr = std::unique_ptr<geometry_utils::TransformListener>(new geometry_utils_ros::TransformListenerRos());
      if (!dynamic_cast<free_gait::AdapterAnymal*>(this->adapter_.get())->setTfListener(tfListenerPtr)) {
        return false;
      }
    } catch (...) {
      MELO_ERROR_STREAM("[FreeGaitRos::create] Could not cast adapter!");
      return false;
    }

    // Add all available actions to the list
    if (!getAvailableActions_.waitForExistence(ros::Duration(5.0))) {
      MELO_WARN("Could not connection free_gait_action_loader.")
    } else {
      std::vector<std::string> actionIds;
      if (!getAvailableActions(actionIds)) {
        MELO_WARN("Could not get actions from free_gait_action_loader.")
      }

      // Add operation modes
      this->setAvailableOperationModesForReferenceType(anymal_motion_control::ReferenceType::ACTION, actionIds);
      this->setAvailableOperationModesForReferenceType(anymal_motion_control::ReferenceType::POSE, locomotionPlannerOperationModes_);
      this->setAvailableOperationModesForReferenceType(anymal_motion_control::ReferenceType::TWIST, locomotionPlannerOperationModes_);
    }

    stairsToClimbNamePtr_ = FreeGait::createAnymalParameter(std::string(), std::string(), std::string(), "staircase_description_file_name",
                                                            "stair_climbing", robot_control::ParameterMutability::DYNAMIC);
    stairsToClimbDirectionPtr_ = FreeGait::createAnymalParameter(std::string(), std::string(), std::string(), "climbing_orientation",
                                                                 "stair_climbing", robot_control::ParameterMutability::DYNAMIC);
    return true;
  }

  bool initialize() override {
    if (!FreeGait::initialize()) {
      MELO_ERROR("Could not initialize FreeGait.");
      return false;
    }

    initializePublishers();

    // Start logging in a worker thread.
    robot_control::WorkerOptions publishWorkerOptions;
    publishWorkerOptions.autostart_ = false;
    publishWorkerOptions.frequency_ = 30.0;
    publishWorkerOptions.name_ = this->getName() + "_publisher";
    publishWorkerOptions.priority_ = 0;
    publishWorkerOptions.callback_ = boost::bind(&FreeGaitRos<Task>::publishWorker, this, _1);
    publishWorkerHandle_ = this->getWorkerManager().addWorker(publishWorkerOptions);
    this->getWorkerManager().startWorker(publishWorkerHandle_);
    dt_ = this->getTime().getTimeStep();
    return true;
  }

  bool advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) override {
    this->timer_.pinTime("FreeGaitRos::advance()");
    boost::unique_lock<boost::shared_mutex> locomotionControllerlock(mutexLocomotionController_);
    bool result = FreeGait::advance(command, commandMutex);
    this->timer_.splitTime("FreeGaitRos::advance()");

    if (this->isRealRobot()) {
      if (this->timer_.getAverageElapsedTimeMSec("FreeGaitRos::advance()") > 2.0) {
        MELO_WARN_STREAM("FreeGaitRos::advance() used " << this->timer_.getAverageElapsedTimeMSec("FreeGaitRos::advance()") << " ms.");
        MELO_INFO_STREAM(this->timer_);
      }
    }

    return result;
  }

  bool preStop() override {
    actionLoaderClient_->cancelAllGoals();

    MELO_INFO("Pre-stopping anymal_ctrl_free_gait_ros.");
    stepActionServer_->block();
    const std::chrono::duration<int, std::milli> period(10);  // 10 ms
    int timeout_it = 500;                                     // 500 iterations, 5 sec timeout
    while (this->executor_->getQueue().active() && this->isRunning()) {
      std::this_thread::sleep_for(period);
      timeout_it--;
      if (timeout_it <= 0) {
        MELO_ERROR("[FreeGaitRos::preStop] Safe stop timeout!");
        break;
      }
    }
    if (stepActionServer_->isActive()) {
      stepActionServer_->setAborted();
    }

    this->getWorkerManager().cancelWorker(publishWorkerHandle_);
    locomotionPlanner_->shutdown();
    stepActionServer_->shutdown();
    resetService_.shutdown();
    checkStateEstimatorService_.shutdown();
    pauseService_.shutdown();
    stopService_.shutdown();
    BaseRos::shutdownRos();
    stepActionServer_->unblock();
    return true;
  }

  bool stop() override {
    MELO_INFO("Stopping anymal_ctrl_free_gait_ros.");
    return FreeGait::stop();
  }

 private:
  void initializePublishers() {
    stepActionServer_->start();

    resetService_ = nodeHandle_.advertiseService("/anymal_ctrl_free_gait/reset", &FreeGaitRos::resetServiceCallback, this);

    checkStateEstimatorService_ = nodeHandle_.advertiseService("/anymal_ctrl_free_gait/check_state_estimator",
                                                               &FreeGaitRos::checkStateEstimatorServiceCallback, this);

    pauseService_ = nodeHandle_.advertiseService("/anymal_ctrl_free_gait/pause_execution", &FreeGaitRos::pauseExecution, this);
    stopService_ = nodeHandle_.advertiseService("/anymal_ctrl_free_gait/stop_execution", &FreeGaitRos::stopExecution, this);

    BaseRos::initRos();
  }

  bool publishWorker(const robot_control::WorkerEvent& event) {
    stepActionServer_->update();
    updateAndPublish();
    return true;
  }

  bool updateAndPublish() {
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
      updateMessages();
      BaseRos::updateRos(*(this->locomotionController_));
    }
    publish();
    BaseRos::publishRos();
    return true;
  }

  virtual void updateMessages() {}

  virtual void publish() {}

  bool resetServiceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    MELO_INFO("Resetting Loco Free Gait.");
    boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    response.success = static_cast<uint8_t>(this->softReset(dt_));
    lock.unlock();
    MELO_INFO("Resetting Done.");
    return true;
  }

  bool pauseExecution(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    this->executor_->pause(request.data);
    lock.unlock();
    response.success = static_cast<uint8_t>(true);
    return true;
  }

  bool stopExecution(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
    this->executor_->stop();
    response.success = static_cast<uint8_t>(true);
    if (stepActionServer_->isActive()) {
      stepActionServer_->setPreempted();
    }
    lock.unlock();
    return true;
  }

  bool checkStateEstimatorServiceCallback(any_msgs::Toggle::Request& request, any_msgs::Toggle::Response& response) {
    if (static_cast<bool>(request.enable)) {
      ros::Time startTime = ros::Time::now();
      ros::Duration timeout(15.0);
      ros::Rate rate(3.0);

      while (ros::Time::now() - startTime < timeout) {
        boost::shared_lock<boost::shared_mutex> stateLock(this->getStateMutex());
        bool isStateOk = this->getState().getStatus() == anymal_motion_control::State::StateStatus::STATUS_OK;
        stateLock.unlock();

        if (isStateOk) {
          boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
          this->enableCheckingStateEstimator();
          lock.unlock();
          response.success = static_cast<uint8_t>(true);
          return true;
        }
        rate.sleep();
      }
    } else if (!static_cast<bool>(request.enable)) {
      boost::shared_lock<boost::shared_mutex> lock(mutexLocomotionController_);
      this->disableCheckingStateEstimator();
      lock.unlock();
      response.success = static_cast<uint8_t>(true);
      return true;
    }

    response.success = static_cast<uint8_t>(false);
    return true;
  }

  anymal_motion_control::SwitchResult goToReferenceType(anymal_motion_control::ReferenceType referenceType) override {
    switch (referenceType) {
      case anymal_motion_control::ReferenceType::POSE:
        locomotionPlanner_->setInputToPose();
        break;
      case anymal_motion_control::ReferenceType::TWIST:
        locomotionPlanner_->setInputToTwist();
        break;
      default:
        break;
    }
    return anymal_motion_control::SwitchResult::SWITCHED;
  }

  void goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) override {
    // Required to avoid actions other than stand_up starting w/o estimator
    // TODO(fgiraldez): figure out how/if they could work before MCI w/o estimator
    if (operationMode != "stand_up") {
      this->enableCheckingStateEstimator();
    }

    if (std::find(locomotionPlannerOperationModes_.begin(), locomotionPlannerOperationModes_.end(), operationMode) !=
        locomotionPlannerOperationModes_.end()) {
      if (switchToLocomotionPlannerOperationMode(operationMode)) {
        this->adapter_->setAnalyticInverseKinematics();
        action->setSucceeded(anymal_motion_control::SwitchResult::SWITCHED);
        MELO_DEBUG("Locomotion Planner started successfully");
      } else {
        action->setAborted(anymal_motion_control::SwitchResult::ERROR);
        MELO_ERROR("Could not switch to Locomotion Planner");
      }
      return;
    } else {
      locomotionPlanner_->shutdown();
      this->adapter_->setIterativeInverseKinematics();
    }

    free_gait_msgs::ExecuteActionGoal goal;
    goal.action_id = operationMode;
    // In case of operation mode == stair_climbing, expect stairsToClimbNamePtr_ and stairsToClimbDirectionPtr_
    // to be provided.
    if (operationMode == "stair_climbing") {
      // Update values manually in the case that transition is called when already in stair_climbing mode.
      stairsToClimbNamePtr_->updateValue();
      stairsToClimbDirectionPtr_->updateValue();
      std::string stairsToClimbName = stairsToClimbNamePtr_->getValue();
      std::string stairsToClimbDirection = stairsToClimbDirectionPtr_->getValue();
      stairsToClimbNamePtr_->stageValue(std::string());       // Clear for next time.
      stairsToClimbDirectionPtr_->stageValue(std::string());  // Clear for next time.

      if (!stairsToClimbName.empty() && !stairsToClimbDirection.empty()) {
        MELO_INFO("Setting staircase_description_file_name: [%s],  climbing_orientation: [%s].", stairsToClimbName.c_str(),
                  stairsToClimbDirection.c_str())
        try {
          param_io::setParam(nodeHandle_, "/stair_climbing/staircase_description_file_name", stairsToClimbName);
          param_io::setParam(nodeHandle_, "/stair_climbing/climbing_orientation", stairsToClimbDirection);
        } catch (...) {
          action->setAborted(anymal_motion_control::SwitchResult::ERROR);
          MELO_ERROR("Could not write staircase_description_file_name or climbing_orientation into ROS.");
          return;
        }
      }
    }

    if (!actionLoaderClient_->waitForServer(ros::Duration(1.0))) {
      action->setAborted(anymal_motion_control::SwitchResult::ERROR);
      MELO_ERROR("Could not connect to action server.")
      return;
    }

    actionLoaderClient_->sendGoal(goal);

    while (!actionLoaderClient_->waitForResult(ros::Duration(0.1))) {
      // Stop action if preempted or aborted from a higher level
      if (action->isPreemptionRequested()) {
        actionLoaderClient_->cancelAllGoals();
      }
    }

    if (actionLoaderClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      if (actionLoaderClient_->getResult()->status != free_gait_msgs::ExecuteActionResult::RESULT_SUCCEEDED) {
        action->setAborted(anymal_motion_control::SwitchResult::ERROR);
        MELO_ERROR_STREAM("Could not execute action '" + operationMode + "'.");
      } else {
        action->setSucceeded(anymal_motion_control::SwitchResult::SWITCHED, "Successfully executed action '" + operationMode + "'.");
        MELO_DEBUG_STREAM("Successfully executed action '" + operationMode + "'.");
      }
    } else if (actionLoaderClient_->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
      if (actionLoaderClient_->getResult()->status != free_gait_msgs::ExecuteActionResult::RESULT_PREEMPTED) {
        action->setAborted(anymal_motion_control::SwitchResult::ERROR);
        MELO_ERROR_STREAM("Could not preempt action '" + operationMode + "'.");
      } else {
        action->setPreempted(anymal_motion_control::SwitchResult::ERROR);
        MELO_DEBUG_STREAM("Execute action '" + operationMode + "' was preempted.");
      }
    } else {
      action->setAborted(anymal_motion_control::SwitchResult::ERROR);
      MELO_ERROR_STREAM("Execute action '" + operationMode + "' was aborted.");
    }
  }

  std::vector<std::string> getLocomotionPlannerOperationModes() {
    std::vector<std::string> operationModes;
    for (const auto& operationModeConfigPair : operationModeToLocomPlannerConfigMap_) {
      operationModes.emplace_back(operationModeConfigPair.first);
    }
  }

  bool switchToLocomotionPlannerOperationMode(const std::string& operationMode) {
    const auto locomPlannerConfigIt = operationModeToLocomPlannerConfigMap_.find(operationMode);
    if (locomPlannerConfigIt == operationModeToLocomPlannerConfigMap_.end()) {
      MELO_ERROR_STREAM("Could not find Locomotion Planner configuration for operation mode " << operationMode << ".");
      return false;
    }
    locomotionPlanner_->shutdown();

    const auto locomPlannerConfig = (*locomPlannerConfigIt).second;
    locomotionPlanner_->setConfiguration(locomPlannerConfig);
    if (!locomotionPlanner_->initialize()) {
      MELO_ERROR_STREAM("Could not initialize Locomotion Planner with config " << locomPlannerConfig << ".");
      return false;
    }
    // TODO(fgiraldez): parameter currently only exists to update joystick of twist limits. It should be made adjustable in the future (via
    // callback).
    locomotion_planner::PlanarTwist maxTwistCommandSlow = locomotionPlanner_->getMaxTwistCommandSlow();
    locomotion_planner::PlanarTwist maxTwistCommandFast = locomotionPlanner_->getMaxTwistCommandFast();
    maxLinearVelocityLocomotionPlannerPtr_ =
        FreeGait::createAnymalParameter(maxTwistCommandFast.x(), maxTwistCommandSlow.x(), locomotionPlanner_->getMaxTwistCommandFast().x(),
                                        "max_linear_vel", operationMode, robot_control::ParameterMutability::DYNAMIC);
    maxAngularVelocityLocomotionPlannerPtr_ = FreeGait::createAnymalParameter(
        locomotionPlanner_->getMaxTwistCommandFast().z(), locomotionPlanner_->getMaxTwistCommandSlow().z(),
        locomotionPlanner_->getMaxTwistCommandFast().z(), "max_angular_vel", operationMode, robot_control::ParameterMutability::DYNAMIC);
    return true;
  }

  bool getAvailableActions(std::vector<std::string>& actionIds) {
    free_gait_msgs::GetActions actionsSrv;
    if (!getAvailableActions_.call(actionsSrv)) {
      MELO_WARN("Could not get available collections")
      return false;
    }

    for (const auto& action : actionsSrv.response.actions) {
      actionIds.emplace_back(action.id);
    }

    return true;
  }

  bool getDirectoryContentsService(any_msgs::GetStrings::Request& /*request*/, any_msgs::GetStrings::Response& response) {
    using boost::filesystem::directory_iterator;
    const boost::filesystem::path directoryPath(
        param_io::param<std::string>(nodeHandle_, "/stair_climbing/staircase_description_folder_path", ""));
    const bool isDirectory = is_directory(directoryPath);
    if (isDirectory) {
      for (const auto& file : boost::make_iterator_range(directory_iterator(directoryPath), {})) {
        if (file.path().extension() == ".yaml") {
          response.data.push_back(file.path().filename().string());
        }
      }
    }
    response.success = static_cast<uint8_t>(isDirectory);
    return true;
  }

 protected:
  // Singleton because having multiple action servers on the same topic is problematic.
  std::unique_ptr<free_gait::FreeGaitActionServer> stepActionServer_;

 private:
  std::unique_ptr<actionlib::SimpleActionClient<free_gait_msgs::ExecuteActionAction>> actionLoaderClient_;
  // TODO(fgiraldez): When ROSlaunch actions are integrated with MCI, move Locomotion Planner one abstraction level lower (ROS-independent)
  std::unique_ptr<locomotion_planner::LocomotionPlannerRos> locomotionPlanner_;
  // TODO(fgiraldez): load operation modes and mappings from file
  std::vector<std::string> locomotionPlannerOperationModes_ = {"locomotion_planner", "locomotion_planner_creep",
                                                               "locomotion_planner_blind"};
  std::map<std::string, std::string> operationModeToLocomPlannerConfigMap_ = {
      {"locomotion_planner", "default"}, {"locomotion_planner_creep", "creep"}, {"locomotion_planner_blind", "blind"}};
  ros::ServiceClient getAvailableActions_;
  double dt_;
  robot_control::WorkerHandle publishWorkerHandle_;
  ros::ServiceServer resetService_;
  ros::ServiceServer pauseService_;
  ros::ServiceServer stopService_;
  ros::ServiceServer checkStateEstimatorService_;
  ros::ServiceServer getDirectoryContentsService_;
  boost::shared_mutex mutexLocomotionController_;
  robot_control::ParameterPtr<std::string> stairsToClimbNamePtr_;
  robot_control::ParameterPtr<std::string> stairsToClimbDirectionPtr_;
  robot_control::ParameterPtr<double> maxLinearVelocityLocomotionPlannerPtr_;
  robot_control::ParameterPtr<double> maxAngularVelocityLocomotionPlannerPtr_;
};

}  // namespace anymal_ctrl_free_gait
