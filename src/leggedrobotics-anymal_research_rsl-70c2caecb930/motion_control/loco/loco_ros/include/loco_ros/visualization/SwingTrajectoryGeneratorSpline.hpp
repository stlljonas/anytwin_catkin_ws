/*
 * SwingTrajectoryGeneratorSpline.hpp
 *
 *  Created on: Jun 4, 2016
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include <loco/swing_trajectory_generation/SwingTrajectoryGeneratorSpline.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// std
#include <string>

namespace loco_ros {

class SwingTrajectoryGeneratorSpline {
 public:
  SwingTrajectoryGeneratorSpline();
  virtual ~SwingTrajectoryGeneratorSpline();
  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);
  bool shutdown();
  bool update(const loco::SwingTrajectoryGeneratorSpline& generator, const loco::TorsoBase& torso);
  bool visualize();

 protected:
  ros::Publisher publisher_;
  visualization_msgs::MarkerArray markerArray_;
};

} /* namespace loco_ros */
