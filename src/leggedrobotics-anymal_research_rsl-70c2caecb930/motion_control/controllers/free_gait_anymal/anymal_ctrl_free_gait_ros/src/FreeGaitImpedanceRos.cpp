/*
 * FreeGaitImpedanceRos.cpp
 *
 *  Created on: Nov 11, 2016
 *      Author: Christian Gehring
 */

#include <pluginlib/class_list_macros.hpp>

#include "anymal_ctrl_free_gait_ros/FreeGaitImpedanceRos.hpp"

// export controller plugin
PLUGINLIB_EXPORT_CLASS(anymal_ctrl_free_gait::FreeGaitImpedanceRos, robot_control::ControllerInterface)