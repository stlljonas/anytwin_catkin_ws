/*
 * AdapterRosInitializerAnymal.cpp
 *
 *  Created on: Nov 29, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */


#include "free_gait_anymal_common/AdapterRosInterfaceAnymal.hpp"

#include "free_gait_anymal_common/AdapterAnymal.hpp"

// ROS
#include <pluginlib/class_list_macros.h>

// any measurements ros
#include <any_measurements_ros/any_measurements_ros.hpp>

// Geometry utils ROS
#include<geometry_utils_ros/TransformListenerRos.hpp>

// Anymal model
#include <anymal_model/AnymalState.hpp>

// Anymal model ros
#include <anymal_model_ros/conversions.hpp>

namespace free_gait {

AdapterRosInterfaceAnymal::AdapterRosInterfaceAnymal()
    : AdapterRosInterfaceBase(),
      isReady_(false)
{
}

bool AdapterRosInterfaceAnymal::subscribeToRobotState(const std::string& robotStateTopic)
{
  robotStateSubscriber_ = nodeHandle_->subscribe(robotStateTopic, 1, &AdapterRosInterfaceAnymal::robotStateCallback, this);
  return true;
}

void AdapterRosInterfaceAnymal::unsubscribeFromRobotState()
{
  robotStateSubscriber_.shutdown();
}

const std::string AdapterRosInterfaceAnymal::getRobotStateMessageType()
{
  return ros::message_traits::datatype<anymal_msgs::AnymalState>();
}

bool AdapterRosInterfaceAnymal::isReady() const
{
  return isReady_;
}

bool AdapterRosInterfaceAnymal::initializeAdapter(AdapterBase& adapter) const
{
  anymal_model::AnymalModel* anymalModel = new anymal_model::AnymalModel();
  anymalModel->initializeFromUrdf(robotDescriptionUrdfString_);
  try {
    auto tfListenerPtr = std::unique_ptr<geometry_utils_ros::TransformListener>(new geometry_utils_ros::TransformListenerRos());
    dynamic_cast<AdapterAnymal&>(adapter).initialize(anymalModel, tfListenerPtr);
  } catch (...) {
    MELO_ERROR_STREAM("[AdapterRosInterfaceAnymal::initializeAdapter] Could not cast adapter!");
    return false;
  }
  return true;
}

bool AdapterRosInterfaceAnymal::updateAdapterWithRobotState(AdapterBase& adapter) const
{
  if (!isReady_) return false;
  try {
    AdapterAnymal& anymalAdapter = dynamic_cast<AdapterAnymal&>(adapter);
    anymal_model::AnymalModel& model = anymalAdapter.getAnymalModel();
    anymal_model_ros::fromRos(robotState_, model);
    return true;
  } catch (...) {
    MELO_ERROR_STREAM("[AdapterRosInterfaceAnymal::updateAdapterWithRobotState] Could not cast adapter!");
    return false;
  }
}

void AdapterRosInterfaceAnymal::robotStateCallback(const anymal_msgs::AnymalState& robotState)
{
  robotState_ = robotState;
  isReady_ = true;
}

} /* namespace free_gait */

// Declare the AdapterRosInitializerAnymal as a Free Gait Adapter ROS initializer class.
PLUGINLIB_EXPORT_CLASS(free_gait::AdapterRosInterfaceAnymal, free_gait::AdapterRosInterfaceBase)
