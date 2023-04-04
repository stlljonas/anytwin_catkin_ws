/*
 * FinalComBox.hpp
 *
 *  Created on: Feb 1, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// loco
#include <loco/common/typedefs.hpp>
#include <zmp_optimizer/zmp_optimizer.hpp>
#include "zmp_optimizer/MotionPlan.hpp"

// loco ros
#include <loco_ros/loco_ros.hpp>

// robot utils
#include <robot_utils/geometry/geometry.hpp>

// stl
#include <string>


namespace anymal_ctrl_dynamic_gaits_ros {

class FinalComBox {
 public:
  FinalComBox() = default;
  virtual ~FinalComBox() = default;

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);
  bool shutdown();

  bool update(const zmp::MotionPlan& motionPlan);
  bool publish();

  unsigned int getNumSubscribers() const;

 protected:
  ros::NodeHandle nodeHandle_;
  visualization_msgs::Marker box_;
  ros::Publisher publisher_;
};

} /* namespace anymal_ctrl_dynamic_gaits_ros */
