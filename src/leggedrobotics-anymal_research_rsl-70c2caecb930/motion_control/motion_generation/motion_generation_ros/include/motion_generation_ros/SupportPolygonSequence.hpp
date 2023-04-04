/*
 * SupportPolygonSequence.hpp
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
#include <zmp_optimizer/zmp_optimizer.hpp>
#include "zmp_optimizer/MotionPlan.hpp"

// loco ros
#include <loco_ros/loco_ros.hpp>

// robot utils
#include <robot_utils/geometry/geometry.hpp>

// stl
#include <string>


namespace anymal_ctrl_dynamic_gaits_ros {

class SupportPolygonSequence {
 public:
  SupportPolygonSequence();
  virtual ~SupportPolygonSequence();

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);
  bool shutdown();

  bool update(const zmp::MotionPlan& motionPlan);
  bool publish();

  void setColorVector(std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>>& colors);
  void setColorVectorId(unsigned int colorId);

  unsigned int getNumSubscribers() const;

 protected:
  ros::NodeHandle nodeHandle_;
  visualization_msgs::MarkerArray supportPolygonSequence_;
  ros::Publisher publisher_;
  std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>> colors_;
  unsigned int colorId_;
};

} /* namespace loco */
