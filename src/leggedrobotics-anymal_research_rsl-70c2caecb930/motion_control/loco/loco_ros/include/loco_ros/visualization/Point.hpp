/*
 * Point.hpp
 *
 *  Created on: Mar 1, 2017
 *      Author: Gabriel Hottiger, Dario Bellicoso
 */

#pragma once

// ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// loco
#include "loco/common/typedefs.hpp"

// loco ros
#include "loco_ros/loco_ros.hpp"
#include "loco_ros/Color.hpp"

namespace loco_ros {

class Point
{
 public:
  explicit Point();
  virtual ~Point();

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);

  void addPoint(const loco::Position& point, const double scale, const Color& color = Color(ColorEnum::BLUE), const std::string ns = "points");

  void updatePoint(const loco::Position& point, const unsigned int id);

  void clearPoints();

  void shutdown();

  bool visualize();

 private:
  ros::NodeHandle nodeHandle_;
  visualization_msgs::MarkerArray markerArray_;
  ros::Publisher markerArrayPublisher_;
};

} /* namespace loco_ros */
