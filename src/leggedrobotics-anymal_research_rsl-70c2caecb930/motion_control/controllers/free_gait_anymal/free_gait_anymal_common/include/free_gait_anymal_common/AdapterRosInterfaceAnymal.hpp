/*
 * AdapterRosInterfaceAnymal.hpp
 *
 *  Created on: Nov 29, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/executor/AdapterBase.hpp>
#include "free_gait_ros/AdapterRosInterfaceBase.hpp"

// Anymal msgs
#include <anymal_msgs/AnymalState.h>

// ROS geometry msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace free_gait {

class AdapterRosInterfaceAnymal : public AdapterRosInterfaceBase
{
 public:
  AdapterRosInterfaceAnymal();
  ~AdapterRosInterfaceAnymal() override = default;

  bool subscribeToRobotState(const std::string& robotStateTopic);
  void unsubscribeFromRobotState();
  const std::string getRobotStateMessageType();

  //! Update adapter.
  bool isReady() const;
  bool initializeAdapter(AdapterBase& adapter) const;
  bool updateAdapterWithRobotState(AdapterBase& adapter) const;

 private:
  void robotStateCallback(const anymal_msgs::AnymalState& robotState);

  ros::Subscriber robotStateSubscriber_;
  anymal_msgs::AnymalState robotState_;
  bool isReady_;
};

} /* namespace free_gait */
