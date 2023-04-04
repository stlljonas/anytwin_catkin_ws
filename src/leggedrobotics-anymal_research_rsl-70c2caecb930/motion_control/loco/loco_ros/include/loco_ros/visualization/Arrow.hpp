/*
 * Arrow.hpp
 *
 *  Created on: Apr 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// loco
#include "loco/common/typedefs.hpp"

// loco ros
#include "loco_ros/loco_ros.hpp"

namespace loco_ros {

class Arrow
{
 public:
  explicit Arrow();
  virtual ~Arrow();

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);

  void addArrow(const loco::Position& arrowStart, const loco::Position& arrowEnd, unsigned int id = 0);
  void addArrow(const loco::Position& arrowStart, const loco::Vector& arrowComponents, unsigned int id = 0);

  void updateArrow(const loco::Position& arrowStart, const loco::Position& arrowEnd, unsigned int id);
  void updateArrow(const loco::Position& arrowStart, const loco::Vector& arrowComponents, unsigned int id);

  void clearArrows();

  void shutdown();

  bool visualize();

  void setColorVector(std::vector<loco::Vector>& colors);

 private:
  ros::NodeHandle nodeHandle_;
  visualization_msgs::MarkerArray markerArray_;
  ros::Publisher markerArrayPublisher_;
  std::vector<loco::Vector> colors_;
};

} /* namespace loco_ros */
