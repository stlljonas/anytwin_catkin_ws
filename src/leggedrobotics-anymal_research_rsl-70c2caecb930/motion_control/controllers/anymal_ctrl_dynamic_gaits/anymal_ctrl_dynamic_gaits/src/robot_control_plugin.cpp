/*
 * anymal_ctrl_dynamic_gaits_plugin.cpp
 *
 *  Created on: Mar 28, 2018
 *      Author: C. Dario Bellicoso
 */

#include <pluginlib/class_list_macros.hpp>

// controller
#include "anymal_ctrl_dynamic_gaits/DynamicGaitsController.hpp"

// export controller plugin
PLUGINLIB_EXPORT_CLASS(anymal_ctrl_dynamic_gaits::DynamicGaitsController, robot_control::ControllerInterface)