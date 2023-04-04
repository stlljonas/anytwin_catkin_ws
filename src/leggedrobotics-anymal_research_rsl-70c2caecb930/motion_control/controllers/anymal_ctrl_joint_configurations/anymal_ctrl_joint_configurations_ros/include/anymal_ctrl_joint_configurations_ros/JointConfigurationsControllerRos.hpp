/*!
 * @file    JointConfigurationsControllerRos.hpp
 * @author  Alexander Reske
 * @brief   A controller with ROS interface for the controller JointConfigurationsController.
 */

#pragma once

// stl
#include <memory>

// ros
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

// anymal_ctrl_joint_configurations
#include "anymal_ctrl_joint_configurations/JointConfigurationsController.hpp"
#include "anymal_ctrl_joint_configurations_msgs/JointConfigurationAction.h"

namespace anymal_ctrl_joint_configurations_ros {

class JointConfigurationsControllerRos : public anymal_ctrl_joint_configurations::JointConfigurationsController {
 public:
  using Base = anymal_ctrl_joint_configurations::JointConfigurationsController;
  using JointConfigurationActionServer = actionlib::SimpleActionServer<anymal_ctrl_joint_configurations_msgs::JointConfigurationAction>;

  //! Construct JointConfigurationsControllerRos.
  JointConfigurationsControllerRos();

  //! Destruct JointConfigurationsControllerRos.
  ~JointConfigurationsControllerRos() override = default;

 protected:
  /*!
   * Initialize controller JointConfigurationsControllerRos.
   * @returns true if successful.
   */
  bool initialize() override;

  /*!
   * Reset controller JointConfigurationsControllerRos.
   * @returns true if successful.
   */
  bool reset() override;

  /*!
   * Pre-stop controller JointConfigurationsControllerRos.
   * @returns true if successful.
   */
  bool preStop() override;

  /*!
   * Callback for when the JointConfigurationActionServer receives a new goal and passes it on.
   */
  void jointConfigurationExecuteCallback(const anymal_ctrl_joint_configurations_msgs::JointConfigurationGoalConstPtr& goal);

 private:
  /*!
   * Initialize ROS.
   */
  void initRos();

  /*!
   * Shutdown ROS.
   */
  void shutdownRos();

  // Member variables
  ros::NodeHandle nodeHandle_;
  std::unique_ptr<JointConfigurationActionServer> jointConfigurationActionServer_;
};

} /* namespace anymal_ctrl_joint_configurations_ros */
