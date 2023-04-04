/*
 * SupportPolygon.hpp
 *
 *  Created on: Sep 3, 2015
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include <loco/common/torso/TorsoBase.hpp>
#include <loco/common/legs/Legs.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// std
#include <string>

namespace loco_ros {

/*! Draws the support polygon of the current robot configuration.
 *
 */
class SupportPolygon
{
 public:
  SupportPolygon();
  virtual ~SupportPolygon();
  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);
  bool shutdown();
  bool update(const loco::TorsoBase& torso, const loco::Legs& legs, bool projectOnXYPlane = false);
  bool publish();
  unsigned int getNumSubscribers() const;
protected:
  ros::NodeHandle nodeHandle_;
  visualization_msgs::Marker supportPolygon_;
  ros::Publisher publisher_;

};

} /* namespace loco_ros */
