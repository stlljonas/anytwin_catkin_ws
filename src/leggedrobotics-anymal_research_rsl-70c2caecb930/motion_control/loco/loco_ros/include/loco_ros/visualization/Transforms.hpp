/*
 * Transforms.hpp
 *
 *  Created on: Jan 19, 2017
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include <loco/common/torso/TorsoBase.hpp>
#include <loco/common/TerrainModelBase.hpp>
#include <loco/common/FootprintGenerator.hpp>

// ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <kindr_ros/kindr_ros.hpp>

namespace loco_ros {

class Transforms {
 public:
  Transforms() {};
  virtual ~Transforms() {};

  bool initialize(ros::NodeHandle& nodeHandle) {
    nodeHandle_ = nodeHandle;
    // initialize transforms
    transformControlFrame_.frame_id_ = "odom";
    transformControlFrame_.child_frame_id_ = "control";
    transformControlFrame_.setIdentity();
    footprintPose_.frame_id_ = "odom";
    footprintPose_.child_frame_id_ = "loco_footprint";
    footprintPose_.setIdentity();
    return true;
  }

  bool shutdown() {
    return true;
  }

  bool update(const loco::WholeBody& wholeBody, const loco::TerrainModelBase& terrainModel, const loco::HeadingGenerator& headingGenerator) {
    const loco::Position& positionWorldToControlInWorldFrame = wholeBody.getTorso().getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame();
    const loco::RotationQuaternion& orientationWorldToControl = wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();

    const ros::Time timestamp = ros::Time::now();

    // update the loco related tfs
    kindr_ros::convertToRosTf(
        loco::Pose(positionWorldToControlInWorldFrame, orientationWorldToControl.inverted()),
                   transformControlFrame_);
    transformControlFrame_.stamp_ = timestamp;

    if (footprintGenerator_.update(wholeBody.getLegs(), headingGenerator)) {
      kindr_ros::convertToRosTf(footprintGenerator_.getFootprintPose(), footprintPose_);
      footprintPose_.stamp_ = timestamp;
    }
    return true;
  }

  bool publish() {
    transformBroadcaster_.sendTransform(transformControlFrame_);
    transformBroadcaster_.sendTransform(footprintPose_);
    return true;
  }

protected:
  ros::NodeHandle nodeHandle_;
  tf::TransformBroadcaster transformBroadcaster_;
  tf::StampedTransform transformControlFrame_;
  tf::StampedTransform footprintPose_;
  loco::FootprintGenerator footprintGenerator_;
};


} /* namespace loco_ros */
