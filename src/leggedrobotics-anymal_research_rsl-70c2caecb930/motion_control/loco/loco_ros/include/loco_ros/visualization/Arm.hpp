/*
 * Arm.hpp
 *
 *  Created on: Sep 3, 2015
 *      Author: Christian Gehring, Peter Fankhauser
 */

#pragma once

// loco
#include <loco/common/arms/ArmBase.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <kindr_msgs/VectorAtPosition.h>

// std
#include <string>

#include <loco_ros/loco_ros.hpp>
#include <loco_ros/visualization/Point.hpp>
#include <loco/common/legs/Legs.hpp>
#include <loco/common/arms/ArmBase.hpp>
#include <loco/common/TerrainModelBase.hpp>

namespace loco_ros {

class Arm
{
 public:
    Arm();
    virtual ~Arm();

    bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);bool shutdown();bool update(const loco::ArmBase& arm);bool publish();
 protected:
    Point armPoints_;

};

} /* namespace loco_ros */
