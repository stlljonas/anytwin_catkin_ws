/*!
 * @file    JointConfigurationsControllerRos.cpp
 * @author  Alexander Reske
 * @brief   A controller with ROS interface for the controller JointConfigurationsController.
 */

// stl
#include <functional>
#include <mutex>

// anymal_ctrl_joint_configurations_ros
#include "anymal_ctrl_joint_configurations_ros/JointConfigurationsControllerRos.hpp"

namespace anymal_ctrl_joint_configurations_ros {

JointConfigurationsControllerRos::JointConfigurationsControllerRos()
    : Base(), nodeHandle_(ros::NodeHandle("~")), jointConfigurationActionServer_(nullptr) {}

bool JointConfigurationsControllerRos::initialize() {
  if (!Base::initialize()) {
    return false;
  }
  initRos();
  return true;
}

bool JointConfigurationsControllerRos::reset() {
  return initialize();
}

bool JointConfigurationsControllerRos::preStop() {
  shutdownRos();
  return Base::preStop();
}

void JointConfigurationsControllerRos::jointConfigurationExecuteCallback(
    const anymal_ctrl_joint_configurations_msgs::JointConfigurationGoalConstPtr& goal) {
  anymal_ctrl_joint_configurations_msgs::JointConfigurationResult result;
  result.goal_reached = anymal_ctrl_joint_configurations_msgs::JointConfigurationResult::NA;
  if (Base::getControllerState() != Base::States::Standby) {
    MELO_WARN("Can only start custom joint configurations action from state 'standby'.");
    jointConfigurationActionServer_->setAborted(result);
    return;
  }
  if (Base::getMode() != "custom") {
    MELO_WARN("Can only start custom joint configurations action in mode 'custom'.");
    jointConfigurationActionServer_->setAborted(result);
    return;
  }

  Base::JointVector jointPositions(goal->joint_positions.data());
  Base::getCustomJointConfiguration().contact_ = true;
  Base::getCustomJointConfiguration().jointPositions_ = jointPositions;
  {
    std::lock_guard<std::mutex> controllerLock(getControllerMutex());
    Base::getControllerState() = Base::States::Planning;
  }

  ros::Rate rate(10);
  while ((Base::getControllerState() == Base::States::Planning) || (Base::getControllerState() == Base::States::Active)) {
    if (jointConfigurationActionServer_->isPreemptRequested()) {
      MELO_WARN_STREAM("Received request to preempt operation mode 'custom'.");
      std::lock_guard<std::mutex> controllerLock(getControllerMutex());
      Base::getControllerState() = Base::States::Preempted;
      break;
    }
    rate.sleep();
    if (Base::getMode() != "custom") {
      MELO_WARN("Changed operation mode while executing operation mode 'custom'.");
      jointConfigurationActionServer_->setAborted(result);
      return;
    }
  }
  // There are three main reasons why the while loop terminates: Base::States::Success, Base::States::Preempted or Base::States::Error
  // Base::States::Standby (indicating that mode custom has been triggered again) and anything else leads to a not assigned (NA) result
  if (Base::getControllerState() == Base::States::Success) {
    result.goal_reached = anymal_ctrl_joint_configurations_msgs::JointConfigurationResult::SUCCESS;
    jointConfigurationActionServer_->setSucceeded(result);
  } else if (Base::getControllerState() == Base::States::Preempted) {
    result.goal_reached = anymal_ctrl_joint_configurations_msgs::JointConfigurationResult::PREEMPTED;
    jointConfigurationActionServer_->setPreempted(result);
    return;
  } else if (Base::getControllerState() == Base::States::Error) {
    result.goal_reached = anymal_ctrl_joint_configurations_msgs::JointConfigurationResult::ERROR;
    jointConfigurationActionServer_->setAborted(result);
  } else {
    result.goal_reached = anymal_ctrl_joint_configurations_msgs::JointConfigurationResult::NA;
    jointConfigurationActionServer_->setAborted(result);
  }
}

void JointConfigurationsControllerRos::initRos() {
  jointConfigurationActionServer_ = std::make_unique<JointConfigurationActionServer>(
      nodeHandle_, "/" + this->getName() + "/action",
      std::bind(&JointConfigurationsControllerRos::jointConfigurationExecuteCallback, this, std::placeholders::_1), false);
  jointConfigurationActionServer_->start();
}

void JointConfigurationsControllerRos::shutdownRos() {
  if (jointConfigurationActionServer_ != nullptr) {
    jointConfigurationActionServer_->shutdown();
  }
}

} /* namespace anymal_ctrl_joint_configurations_ros */
