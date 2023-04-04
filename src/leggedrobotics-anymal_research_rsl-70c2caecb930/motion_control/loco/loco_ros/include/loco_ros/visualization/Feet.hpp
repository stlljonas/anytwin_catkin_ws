/*
 * Feet.hpp
 *
 *  Created on: Jan 19, 2017
 *      Author: Christian Gehring, C. Dario Bellicoso
 */

#pragma once

// ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "loco_ros/visualization/ContactForces.hpp"
#include "loco_ros/visualization/ModuleRos.hpp"

// loco
#include <loco/common/legs/Legs.hpp>

namespace loco_ros {

class Feet : public ModuleRos {
 private:
  using Base = ModuleRos;
 public:
  Feet() :
    numLegs_(4),
    forceVectorScale_(1.0),
    plannedFootholdPub_(),
    plannedFootPositionPub_(),
    plannedFootholdMarkers_(),
    plannedFootPositionMarkers_(),
    desiredSwingLinearVelocityAtPositionPairs_(),
    desiredSwingLinearAccelerationAtPositionPairs_()
  {
    publisherRefs_.push_back(plannedFootholdPub_);
    publisherRefs_.push_back(plannedFootPositionPub_);

    for (auto& pair : desiredSwingLinearVelocityAtPositionPairs_) {
      publisherRefs_.push_back(pair.first);
    }

    for (auto& pair : desiredSwingLinearAccelerationAtPositionPairs_) {
      publisherRefs_.push_back(pair.first);
    }
  };

  virtual ~Feet() = default;

  bool initialize(ros::NodeHandle& nodeHandle, bool isSimulation = false) {
    nodeHandle_ = nodeHandle;

    ///////////////////////////////////////////////////////////////////////////////////////
    /// Feet positions
    ///////////////////////////////////////////////////////////////////////////////////////
    plannedFootPositionMarkers_.markers.clear();
    plannedFootPositionMarkers_.markers.push_back(getInitializedReferenceSphereMarker("odom", "planned_foot_position", 0));
    plannedFootPositionMarkers_.markers.push_back(getInitializedReferenceSphereMarker("odom", "planned_foot_position", 1));
    plannedFootPositionMarkers_.markers.push_back(getInitializedReferenceSphereMarker("odom", "planned_foot_position", 2));
    plannedFootPositionMarkers_.markers.push_back(getInitializedReferenceSphereMarker("odom", "planned_foot_position", 3));

    plannedFootholdMarkers_.markers.clear();
    for (unsigned k = 0; k < numLegs_; k++) {
      plannedFootholdMarkers_.markers.push_back(getInitializedReferenceSphereMarker("odom", "planned_foothold", k));
    }

    // Planned and measured foot positions in Cartesian space.
    plannedFootPositionPub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("/loco_ros/planned_foot_positions", 1);
    plannedFootholdPub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("/loco_ros/planned_footholds", 1);

    ///////////////////////////////////////////////////////////////////////////////////////
    /// Feet velocities & accelerations
    ///////////////////////////////////////////////////////////////////////////////////////
    // Desired swing motion: linear velocity.
     desiredSwingLinearVelocityAtPositionPairs_.clear();
     desiredSwingLinearVelocityAtPositionPairs_.push_back(
         std::make_pair(
             nodeHandle_.advertise<kindr_msgs::VectorAtPosition>(
                 "/loco_ros/desired_linear_velocty_lf", 1),
             kindr_msgs::VectorAtPosition()));
     desiredSwingLinearVelocityAtPositionPairs_.push_back(
         std::make_pair(
             nodeHandle_.advertise<kindr_msgs::VectorAtPosition>(
                 "/loco_ros/desired_linear_velocty_rf", 1),
             kindr_msgs::VectorAtPosition()));
     desiredSwingLinearVelocityAtPositionPairs_.push_back(
         std::make_pair(
             nodeHandle_.advertise<kindr_msgs::VectorAtPosition>(
                 "/loco_ros/desired_linear_velocty_lh", 1),
             kindr_msgs::VectorAtPosition()));
     desiredSwingLinearVelocityAtPositionPairs_.push_back(
         std::make_pair(
             nodeHandle_.advertise<kindr_msgs::VectorAtPosition>(
                 "/loco_ros/desired_linear_velocty_rh", 1),
             kindr_msgs::VectorAtPosition()));

     // Desired swing motion: linear acceleration.
     desiredSwingLinearAccelerationAtPositionPairs_.clear();
     desiredSwingLinearAccelerationAtPositionPairs_.push_back(
         std::make_pair(
             nodeHandle_.advertise<kindr_msgs::VectorAtPosition>(
                 "/loco_ros/desired_linear_acceleration_lf", 1),
             kindr_msgs::VectorAtPosition()));
     desiredSwingLinearAccelerationAtPositionPairs_.push_back(
         std::make_pair(
             nodeHandle_.advertise<kindr_msgs::VectorAtPosition>(
                 "/loco_ros/desired_linear_acceleration_rf", 1),
             kindr_msgs::VectorAtPosition()));
     desiredSwingLinearAccelerationAtPositionPairs_.push_back(
         std::make_pair(
             nodeHandle_.advertise<kindr_msgs::VectorAtPosition>(
                 "/loco_ros/desired_linear_acceleration_lh", 1),
             kindr_msgs::VectorAtPosition()));
     desiredSwingLinearAccelerationAtPositionPairs_.push_back(
         std::make_pair(
             nodeHandle_.advertise<kindr_msgs::VectorAtPosition>(
                 "/loco_ros/desired_linear_acceleration_rh", 1),
             kindr_msgs::VectorAtPosition()));

     // Initialize messages.
     for (auto& pair : desiredSwingLinearVelocityAtPositionPairs_) {
       pair.second.header.frame_id = "odom";
       pair.second.position_frame_id = "odom";
       pair.second.type = kindr_msgs::VectorAtPosition::TYPE_VELOCITY;
     }

     for (auto& pair : desiredSwingLinearAccelerationAtPositionPairs_) {
       pair.second.header.frame_id = "odom";
       pair.second.position_frame_id = "odom";
       pair.second.type = kindr_msgs::VectorAtPosition::TYPE_ACCELERATION;
     }

    return true;
  }

  bool update(const loco::Legs& legs) {
    const ros::Time timestamp = ros::Time::now();
    if (plannedFootholdPub_.getNumSubscribers() > 0u) {
      for (const auto& leg : legs) {
        const loco::Position& positionWorldToDesiredFootholdInWorldFrame = leg->getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame();
        const int legId = leg->getId();
        plannedFootholdMarkers_.markers[legId].pose.position.x = positionWorldToDesiredFootholdInWorldFrame.x();
        plannedFootholdMarkers_.markers[legId].pose.position.y = positionWorldToDesiredFootholdInWorldFrame.y();
        plannedFootholdMarkers_.markers[legId].pose.position.z = positionWorldToDesiredFootholdInWorldFrame.z();
        plannedFootholdMarkers_.markers[legId].header.stamp = timestamp;
      }
    }
    if (plannedFootPositionPub_.getNumSubscribers() > 0u) {
      for (const auto& leg : legs) {
        const int legId = leg->getId();
        const loco::Position& positionWorldToDesiredFootInWorldFrame = leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame();
        plannedFootPositionMarkers_.markers[legId].pose.position.x = positionWorldToDesiredFootInWorldFrame.x();
        plannedFootPositionMarkers_.markers[legId].pose.position.y = positionWorldToDesiredFootInWorldFrame.y();
        plannedFootPositionMarkers_.markers[legId].pose.position.z = positionWorldToDesiredFootInWorldFrame.z();

        if ((leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::Motion) ||
            (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::ContactRecovery)) {
          plannedFootPositionMarkers_.markers[legId].color.r = 1.0;
          plannedFootPositionMarkers_.markers[legId].color.g = 0.0;
          plannedFootPositionMarkers_.markers[legId].color.b = 1.0;
        } else {
          plannedFootPositionMarkers_.markers[legId].color.r = 1.0;
          plannedFootPositionMarkers_.markers[legId].color.g = 0.0;
          plannedFootPositionMarkers_.markers[legId].color.b = 0.0;
        }
      }
    }

    // Velocities and Acceleration
    unsigned int legId = 0;
    for (auto& pair : desiredSwingLinearVelocityAtPositionPairs_) {
      if (pair.first.getNumSubscribers() > 0u) {
        const loco::LinearVelocity& velocity = legs.get(legId).getFoot().getStateDesired().getLinearVelocityEndEffectorInWorldFrame();
        const loco::Position& position = legs.get(legId).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        pair.second.header.stamp = timestamp;

        pair.second.vector.x = velocity.x();
        pair.second.vector.y = velocity.y();
        pair.second.vector.z = velocity.z();

        pair.second.position.x = position.x();
        pair.second.position.y = position.y();
        pair.second.position.z = position.z();
      }
      ++legId;
    }

    legId = 0;
    for (auto& pair : desiredSwingLinearAccelerationAtPositionPairs_) {
      if (pair.first.getNumSubscribers() > 0u) {
        const loco::LinearAcceleration& acceleration = legs.get(legId).getFoot().getStateDesired().getLinearAccelerationEndEffectorInWorldFrame();
        const loco::Position& position = legs.get(legId).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        pair.second.header.stamp = timestamp;

        pair.second.vector.x = acceleration.x();
        pair.second.vector.y = acceleration.y();
        pair.second.vector.z = acceleration.z();

        pair.second.position.x = position.x();
        pair.second.position.y = position.y();
        pair.second.position.z = position.z();
      }
      ++legId;
    }

    return true;
  }

  bool publish() {
    publishMsg<visualization_msgs::MarkerArray>(plannedFootPositionPub_, plannedFootPositionMarkers_);
    publishMsg<visualization_msgs::MarkerArray>(plannedFootholdPub_, plannedFootholdMarkers_);

    // Velocity & acceleration
    for (auto& pair : desiredSwingLinearVelocityAtPositionPairs_) {
      publishMsg<kindr_msgs::VectorAtPosition>(pair.first, pair.second);
    }

    for (auto& pair : desiredSwingLinearAccelerationAtPositionPairs_) {
      publishMsg<kindr_msgs::VectorAtPosition>(pair.first, pair.second);
    }

    return true;
  }

protected:
  unsigned int numLegs_;
  ros::NodeHandle nodeHandle_;
  double forceVectorScale_;

  ros::Publisher plannedFootholdPub_;
  ros::Publisher plannedFootPositionPub_;
  visualization_msgs::MarkerArray plannedFootholdMarkers_;
  visualization_msgs::MarkerArray plannedFootPositionMarkers_;

  std::list<std::pair<ros::Publisher, kindr_msgs::VectorAtPosition> > desiredSwingLinearVelocityAtPositionPairs_;
  std::list<std::pair<ros::Publisher, kindr_msgs::VectorAtPosition> > desiredSwingLinearAccelerationAtPositionPairs_;
};


} /* namespace loco_ros */

