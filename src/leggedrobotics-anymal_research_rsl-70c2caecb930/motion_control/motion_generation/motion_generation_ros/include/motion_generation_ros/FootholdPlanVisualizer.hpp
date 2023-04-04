/*
 * FootholdPlanVisualizer.hpp
 *
 *  Created on: Feb 1, 2017
 *      Author: dbellicoso
 */

#pragma once

// ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// loco
#include <loco/common/typedefs.hpp>
#include <loco/foothold_generation/FootholdPlan.hpp>

// loco ros
#include <loco_ros/loco_ros.hpp>
#include <loco_ros/visualization/ModuleRos.hpp>

// robot utils
#include <robot_utils/geometry/geometry.hpp>

// stl
#include <string>


namespace anymal_ctrl_dynamic_gaits_ros {

class FootholdPlanVisualizer : public ModuleRos {
 public:
  FootholdPlanVisualizer();
  virtual ~FootholdPlanVisualizer() = default;

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);

  bool update(const loco::foothold_generator::FootholdPlan& plan);
  bool publish();

  void setColorVector(const std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>>& colors);

 protected:
  ros::NodeHandle nodeHandle_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> footholdBounds_;
  std::pair<ros::Publisher, visualization_msgs::MarkerArray> collisionBounds_;
  std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>> colors_;
};

} /* namespace loco */
