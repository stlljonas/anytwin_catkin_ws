/*
 * rocoma_plugin.cpp
 *
 *  Created on: Mar 28, 2018
 *      Author: C. Dario Bellicoso
 */

// pluginlib
#include <pluginlib/class_list_macros.hpp>

// controller
#include "anymal_ctrl_dynamic_gaits_ros/DynamicGaitsControllerRos.hpp"

//! Export controller tuple as a rocoma plugin.
PLUGINLIB_EXPORT_CLASS(anymal_ctrl_dynamic_gaits_ros::DynamicGaitsControllerRos, robot_control::ControllerInterface);
