/*
 * FrictionCone.hpp
 *
 *  Created on: Nov 17, 2015
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

class FrictionCone
{
 public:
  FrictionCone(double frictionCoefficient, double coneHeight);
  virtual ~FrictionCone();

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic, int numLegs = 4);
  bool shutdown();
  bool update(const loco::Vector& headingInWorldFrame,
              const loco::Legs& legs,
              const loco::TerrainModelBase& terrainModel);
  bool publish();
  unsigned int getNumSubscribers() const;

  void setFrictionCoefficient(double frictionCoefficient);
  double getFrictionCoefficient() const;

 protected:
  ros::Publisher publisher_;
  visualization_msgs::MarkerArray cones_;

  double frictionCoefficient_;
  double coneHeight_;
};

} /* namespace loco_ros */
