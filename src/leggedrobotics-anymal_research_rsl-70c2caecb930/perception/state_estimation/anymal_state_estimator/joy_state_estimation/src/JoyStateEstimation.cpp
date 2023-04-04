/*!
* @file     JoyStateEstimation.cpp
* @author   Linus Isler
* @date     June, 2016
* @brief
*/

#include <anymal_msgs/ResetStateEstimator.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Trigger.h>
#include <any_msgs/SetUInt32.h>
#include <joy_state_estimation/JoyStateEstimation.hpp>



namespace joy_manager {

JoyStateEstimation::JoyStateEstimation() {
  isCalibrating_ = false;
  isTiming_ = false;
  hasCalibrationFinished_ = true;
  hasCalibrationFailed_ = false;
  hasTimerFinished_ = true;
}

JoyStateEstimation::~JoyStateEstimation() {

}

void JoyStateEstimation::init(const ros::NodeHandle& nh,
                              joy_manager::JoyManager* joyManager,
                              const std::string& name) {
  ModuleBase::init(nh, joyManager, name);
  ROS_INFO_STREAM("[" << name_ << "] init()");

  isCalibrating_ = false;
  isTiming_ = false;
  hasCalibrationFinished_ = true;
  hasCalibrationFailed_ = false;
  hasTimerFinished_ = true;

  resetStateEstimatorClient_ = nh_.serviceClient<anymal_msgs::ResetStateEstimator>("/state_estimator/reset");
  resetStateEstimatorHereClient_ = nh_.serviceClient<std_srvs::Trigger>("/state_estimator/reset_here");
  calibrateForceSencorsClient_ = nh_.serviceClient<any_msgs::SetUInt32>("/state_estimator/calibrate_forces");
}

void JoyStateEstimation::cleanup() {
  ModuleBase::cleanup();
  if (isCalibrating_)
    optoforceThread_.join();
  if (isTiming_)
    timerThread_.join();
}

void JoyStateEstimation::processCommand(const std::string& command, std::string* answer) {
  ROS_INFO_STREAM("[" << name_ << "] received command: " << command);
  if (command == "reset")
    resetStateEstimator(answer);
  else if (command == "reset_here")
    resetStateEstimatorHere(answer);
  else if (command == "calibrate_force_sensors")
    calibrateForceSensors(answer);
  else {
    ROS_WARN_STREAM("[" << name_ << "] Unknown command received!");
    *answer = "unknown command";
  }
}

void JoyStateEstimation::resetStateEstimator(std::string* answer) {
  if (resetStateEstimatorClient_.exists()) {
    anymal_msgs::ResetStateEstimator msg;
    msg.request.stamp = ros::Time::now();
    msg.request.pose.orientation.w = 1.0;
    msg.request.pose.orientation.x = 0.0;
    msg.request.pose.orientation.y = 0.0;
    msg.request.pose.orientation.z = 0.0;
    msg.request.pose.position.x = 0.0;
    msg.request.pose.position.y = 0.0;
    msg.request.pose.position.z = 0.0;

    if (resetStateEstimatorClient_.call(msg)) {
      *answer = "success";
      return;
    }
  }
  ROS_WARN_STREAM("[" << name_ << "] Could not reset state estimator!");
  *answer = "error";
}

void JoyStateEstimation::resetStateEstimatorHere(std::string* answer) {
  if (resetStateEstimatorHereClient_.exists()) {
    std_srvs::Trigger msg;
    if (resetStateEstimatorHereClient_.call(msg)) {
      *answer = "success";
      return;
    }
  }
  ROS_WARN_STREAM("[" << name_ << "] Could not reset state estimator here!");
  *answer = "error";
}


void JoyStateEstimation::calibrateForceSensors(std::string* answer) {
  if (!isCalibrating_ && hasTimerFinished_) {
    ROS_INFO_STREAM("[" << name_ << "] Calibrate force sensors");
    if (isTiming_) {
      timerThread_.join();
      isTiming_ = false;
    }
    optoforceThread_ = boost::thread(&JoyStateEstimation::runCalibrationThread, this);
    *answer = "started";
  }
  else if (isCalibrating_ && !hasCalibrationFinished_)
    *answer = "pending";
  else if (hasCalibrationFinished_) {
    if (!isTiming_) {
      optoforceThread_.join();
      isCalibrating_ = false;
      timerThread_ = boost::thread(&JoyStateEstimation::runTimerThread, this);
    }
    if (hasCalibrationFailed_)
      *answer = "failed";
    else
      *answer = "finished";
  }
  else
    *answer = "error";
}


void JoyStateEstimation::runCalibrationThread() {
  isCalibrating_ = true;
  hasCalibrationFinished_ = false;
  hasCalibrationFailed_ = false;

  bool hasFailed = false;

  any_msgs::SetUInt32 any_msg;
  any_msg.request.data = 1000;

  if (calibrateForceSencorsClient_.exists()) {
    if (!calibrateForceSencorsClient_.call(any_msg))
      hasFailed = true;
  }

  if (hasFailed) {
    ROS_WARN_STREAM("[" << name_ << "] Failed to calibrate force sensors!");
    hasCalibrationFailed_ = true;
  }
  else
    ROS_INFO_STREAM("[" << name_ << "] Finished force sensor calibration.");
  hasCalibrationFinished_ = true;
}


void JoyStateEstimation::runTimerThread() {
  isTiming_ = true;
  hasTimerFinished_ = false;
  ros::Duration timer(kTimer_);
  while (nh_.ok() && (timer.toSec() >= 0.0)) {
    ros::Duration(1.0).sleep();
    timer -= ros::Duration(1.0);
  }
  hasTimerFinished_ = true;
}


} // namespace joy_manager

//Declare the navigation module plugin as a JoyManager class
PLUGINLIB_EXPORT_CLASS(joy_manager::JoyStateEstimation, joy_manager::ModuleBase)
