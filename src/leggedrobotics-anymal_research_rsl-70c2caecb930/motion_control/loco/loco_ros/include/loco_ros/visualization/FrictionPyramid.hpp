/*
 * FrictionPyramid.hpp
 *
 *  Created on: Nov 16, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/torso/TorsoBase.hpp>
#include <loco/common/legs/Legs.hpp>
#include <loco/common/TerrainModelBase.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// std
#include <string>

namespace loco_ros {

class FrictionPyramid
{
 public:
  FrictionPyramid(double frictionCoefficient, double pyramidHeight);
  virtual ~FrictionPyramid() = default;

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic, int numLegs = 4);
  bool shutdown();
  virtual bool update(const loco::Vector& headingInWorldFrame,
                      const loco::Legs& legs,
                      const loco::TerrainModelBase& terrainModel);
  bool publish();
  unsigned int getNumSubscribers() const;

  void setFrictionCoefficient(double frictionCoefficient);
  double getFrictionCoefficient() const;

 protected:
  ros::Publisher publisher_;
  visualization_msgs::MarkerArray pyramids_;

  double frictionCoefficient_;
  double pyramidHeight_;
};

} /* namespace loco_ros */
