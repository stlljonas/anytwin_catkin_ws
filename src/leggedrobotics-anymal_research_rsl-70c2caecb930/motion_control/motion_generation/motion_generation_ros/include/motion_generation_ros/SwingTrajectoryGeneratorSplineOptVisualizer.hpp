/*
 * SwingTrajectoryGeneratorSplineOptVisualizer.hpp
 *
 *  Created on: Jun 4, 2016
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include <loco/swing_trajectory_generation/SwingTrajectoryGeneratorSplineOpt.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// std
#include <string>

namespace anymal_ctrl_dynamic_gaits_ros {

class SwingTrajectoryGeneratorSplineOptVisualizer {
 public:
  SwingTrajectoryGeneratorSplineOptVisualizer();
  virtual ~SwingTrajectoryGeneratorSplineOptVisualizer() = default;
  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);
  bool shutdown();
  bool update(const loco::SwingTrajectoryGeneratorSplineOpt& generator);
  bool publish();

 protected:
  ros::Publisher publisher_;
  visualization_msgs::MarkerArray markerArray_;
};

} /* namespace anymal_ctrl_dynamic_gaits_ros */
