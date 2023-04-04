/*
 * FreeGaitActionServer.hpp
 *
 *  Created on: Feb 6, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <string>
#include <atomic>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_msgs/ExecuteStepsAction.h>

#include "free_gait_ros/StepRosConverter.hpp"

namespace free_gait {

class FreeGaitActionServer
{
 public:
  FreeGaitActionServer(ros::NodeHandle& nodeHandle, std::string  name,
                       Executor& executor, AdapterBase& adapter);

  ~FreeGaitActionServer() = default;

  void initialize();
  void start();
  void update();
  void block();
  void unblock();
  void shutdown();

  bool isActive();
  bool isBlocked();
  void goalCallback();
  void preemptCallback();
  void publishFeedback();
  void setSucceeded();
  void setPreempted();
  void setAborted();

 private:
  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;
  free_gait::Executor& executor_;

  //! ROS converter.
  StepRosConverter adapter_;

  //! Action server.
  std::string name_;
  actionlib::SimpleActionServer<free_gait_msgs::ExecuteStepsAction> server_;
  free_gait_msgs::ExecuteStepsActionResult result_;

  //! True if in process of initializing a new goal.
  std::atomic<bool> isInitializingNewGoal_{false};
  //! True if accepted goal will be ignored
  std::atomic<bool> isIgnoringGoal_{false};
  //! True if in process of preempting.
  std::atomic<bool> isPreempting_{false};

  //! True if server is blocked (will not accept any goals)
  std::atomic<bool> isBlocked_{false};

  //! Number of steps of the current goal.
  size_t nStepsInCurrentGoal_ = 0;
};

} /* namespace */
