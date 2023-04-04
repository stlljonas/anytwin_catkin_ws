/*
 * WholeBodyDynamics.hpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// ros
#include <ros/ros.h>

// loco
#include <loco/common/WholeBody.hpp>

// loco ros
#include <loco_ros/loco_ros.hpp>

// anymal model
#include <anymal_model/AnymalModel.hpp>

namespace loco_ros_anymal {

class WholeBodyDynamics {
 public:
  WholeBodyDynamics() = default;
  virtual ~WholeBodyDynamics() = default;

  bool initialize(ros::NodeHandle& nodeHandle);
  bool shutdown();
  bool update(const loco::WholeBody& wholeBody);
  bool publish();

 protected:
  ros::NodeHandle nodeHandle_;
  std::pair<ros::Publisher, visualization_msgs::Marker> centerOfMass_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> inertiaEllipsoids_;

};


} /* namespace loco_ros */

