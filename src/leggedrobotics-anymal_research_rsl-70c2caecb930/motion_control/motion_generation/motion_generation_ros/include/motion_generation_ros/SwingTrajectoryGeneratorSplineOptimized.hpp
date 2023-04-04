/*
 * SwingTrajectoryGeneratorSplineOptimized.hpp
 *
 *  Created on: Jan. 17, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

#include "swing_trajectory_generation/SwingTrajectoryGeneratorSplineOptimized.hpp"

// loco ros
#include <loco_ros/loco_ros.hpp>
#include <loco_ros/visualization/ModuleRos.hpp>

// ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>


// stl
#include <string>

namespace anymal_ctrl_dynamic_gaits_ros {

class SwingTrajectoryGeneratorSplineOptimized  : public ModuleRos {
 public:
  SwingTrajectoryGeneratorSplineOptimized();
  virtual ~SwingTrajectoryGeneratorSplineOptimized();
  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);
  void setColorVector(const std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>>& colors);
  bool update(const std::vector<sto::MotionPlan>& motionPlan);
  bool publish();

 protected:

  bool initPublishers(const std::string& topic);
  bool initMsgs();

  ros::NodeHandle nodeHandle_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedSwingTrajectory0_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedSwingTrajectory1_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedSwingTrajectory2_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> plannedSwingTrajectory3_;
  std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>> colors_;
  unsigned int numOfSamples_;
};

} /* namespace loco_ros */
