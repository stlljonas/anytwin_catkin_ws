/*!
 * @file    robot_control_plugin.cpp
 * @author  Alexander Reske
 * @brief   Plugin export for controller JointConfigurationsControllerRos.
 */

// pluginlib
#include <pluginlib/class_list_macros.hpp>

// robot_control
#include <robot_control/controllers/ControllerInterface.hpp>

// anymal_ctrl_joint_configurations_ros
#include "anymal_ctrl_joint_configurations_ros/JointConfigurationsControllerRos.hpp"

// export controller plugin
PLUGINLIB_EXPORT_CLASS(anymal_ctrl_joint_configurations_ros::JointConfigurationsControllerRos, robot_control::ControllerInterface)