/*
 * Torso.hpp
 *
 *  Created on: Sep 3, 2015
 *      Author: Christian Gehring, Peter Fankhauser
 */

#pragma once

// loco
#include <loco/common/torso/TorsoBase.hpp>
#include <loco/common/legs/Legs.hpp>
#include <loco/common/TerrainModelBase.hpp>

// ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <kindr_msgs/VectorAtPosition.h>

// std
#include <string>

// loco ros
#include <loco_ros/loco_ros.hpp>
#include <loco_ros/visualization/ModuleRos.hpp>

namespace loco_ros {

class Torso : public ModuleRos {
 public:
  Torso();
  virtual ~Torso() = default;

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);
  bool update(const loco::TorsoBase& torso, const loco::Legs& legs, const loco::TerrainModelBase& terrainModel);
  bool publish();

 protected:
  ros::Publisher publisher_;
  visualization_msgs::MarkerArray markerArray_;
  visualization_msgs::Marker* measBaseSphere_;
  visualization_msgs::Marker* measBaseLine_;
  visualization_msgs::Marker* desBaseSphere_;
  visualization_msgs::Marker* desBaseLine_;
  visualization_msgs::Marker* measBaseOnTerrain_;

  // Desired main body motion.
  ros::Publisher desiredBaseLinearVelocityPub_;
  ros::Publisher desiredBaseAngularVelocityPub_;
  ros::Publisher desiredBaseLinearAccelerationPub_;
  ros::Publisher commandedBaseLinearVelocityPub_;
  ros::Publisher commandedBaseAngularVelocityPub_;
  kindr_msgs::VectorAtPosition desiredBaseLinearVelocityMsg_;
  kindr_msgs::VectorAtPosition desiredBaseAngularVelocityMsg_;
  kindr_msgs::VectorAtPosition desiredBaseLinearAccelerationMsg_;
  kindr_msgs::VectorAtPosition commandedBaseLinearVelocityMsg_;
  kindr_msgs::VectorAtPosition commandedBaseAngularVelocityMsg_;
};

} /* namespace loco_ros */
