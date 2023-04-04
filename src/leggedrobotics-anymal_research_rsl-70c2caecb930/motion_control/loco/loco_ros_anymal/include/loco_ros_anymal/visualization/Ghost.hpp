/*
 * Ghost.hpp
 *
 *  Created on: Jun 2, 2016
 *      Author: Christian Gehring, C. Dario Bellicoso
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <anymal_msgs/AnymalState.h>

#include <loco/common/WholeBody.hpp>

namespace loco_ros_anymal {

namespace ghost {

enum StateEnum : unsigned int {
  Desired = 0,
  Measured
};

}

class Ghost {
 public:
  explicit Ghost(ghost::StateEnum stateEnum = ghost::StateEnum::Desired);
  virtual ~Ghost() = default;

  virtual bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);
  bool shutdown();
  bool update(const loco::WholeBody& wholeBody, const ros::Time& timestamp);
  bool publish();
  unsigned int getNumSubscribers() const;

 protected:
  ghost::StateEnum stateEnum_;
  anymal_msgs::AnymalState ghostAnymalStateMsg_;
  ros::Publisher ghostAnymalStatePub_;
};

} /* namespace loco_ros_anymal */
