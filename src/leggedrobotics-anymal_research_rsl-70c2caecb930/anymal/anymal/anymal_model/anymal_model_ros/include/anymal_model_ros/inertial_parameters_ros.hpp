/*!
 * @file     inertial_parameters_ros.hpp
 * @author   Ioannis Mandralis
 * @date     Mar, 2020
 */

#pragma once

// anymal model
#include <anymal_description/AnymalDescription.hpp>
#include <anymal_model/AnymalModel.hpp>

// ros
#include <ros/ros.h>

// stl
#include <string>
#include <vector>
// Param IO
#include <param_io/get_param.hpp>

namespace anymal_model_ros {

// Updates the inertial parameters of the robot from a YAML file.
bool updateInertialParameters(anymal_model::AnymalModel* anymalModel, ros::NodeHandle& nodeHandle);

}  // namespace anymal_model_ros
