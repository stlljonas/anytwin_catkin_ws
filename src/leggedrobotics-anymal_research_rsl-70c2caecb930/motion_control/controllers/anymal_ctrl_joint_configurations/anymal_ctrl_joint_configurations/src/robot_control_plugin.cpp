/*!
 * @file    robot_control_plugin.cpp
 * @author  Alexander Reske
 * @brief   Plugin export for controller JointConfigurationsController.
 */

// pluginlib
#include <pluginlib/class_list_macros.hpp>

// robot_control
#include <robot_control/controllers/ControllerInterface.hpp>

// anymal_ctrl_joint_configurations
#include "anymal_ctrl_joint_configurations/JointConfigurationsController.hpp"

// export controller plugin
PLUGINLIB_EXPORT_CLASS(anymal_ctrl_joint_configurations::JointConfigurationsController, robot_control::ControllerInterface)