/*
 * StateCheckerPublisher.hpp
 *
 *  Created on: Sept 1, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

// ros.
#include <ros/ros.h>

// loco ros.
#include <loco_ros/loco_ros.hpp>

// std msgs.
#include <std_msgs/Float32.h>

namespace anymal_ctrl_dynamic_gaits_ros {

class StateCheckerPublisher {
 public:
  StateCheckerPublisher() = default;
  virtual ~StateCheckerPublisher() = default;

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);
  bool shutdown();

  bool update(bool isSafe);
  bool publish();

  unsigned int getNumSubscribers() const;

 protected:
  ros::NodeHandle nodeHandle_;
  std_msgs::Float32 isSafe_;
  ros::Publisher publisher_;
};

} /* namespace anymal_ctrl_dynamic_gaits_ros */
