/*
 * ContactForces.hpp
 *
 *  Created on: Jan 19, 2017
 *      Author: Christian Gehring, C. Dario Bellicoso, Gabriel Hottiger
 */

#pragma once

// loco_ros
#include "loco_ros/loco_ros.hpp"
#include <loco_ros/visualization/ModuleRos.hpp>

// loco
#include "loco/common/limbs/Limbs.hpp"

// ros
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>


namespace loco_ros {

class ContactForces : public ModuleRos {
 public:
  ContactForces() :
    forceVectorScale_(1.0),
    measuredEndEffectorPositionPub_(),
    measuredEndEffectorPositionMarkers_(),
    desiredContactForceAtPositionPairs_(),
    measuredContactForceAtPositionPairs_()
 {
    publisherRefs_.push_back(measuredEndEffectorPositionPub_);
    for (auto& pair : desiredContactForceAtPositionPairs_) {
      publisherRefs_.push_back(pair.first);
    }
    for (auto& pair : measuredContactForceAtPositionPairs_) {
      publisherRefs_.push_back(pair.first);
    }
 };

  ~ContactForces() override = default;

  bool initialize(ros::NodeHandle& nodeHandle,
                  const std::vector<std::string>& limbNames,
                  const double markerScale = 0.01,
                  const double forceVectorScale = 1.0)
  {
      nodeHandle_ = nodeHandle;
      forceVectorScale_ = forceVectorScale;

      ///////////////////////////////////////////////////////////////////////////////////////
      /// Feet positions
      ///////////////////////////////////////////////////////////////////////////////////////
      measuredEndEffectorPositionMarkers_.markers.clear();
      for(unsigned int i = 0; i < limbNames.size(); ++i) {
        measuredEndEffectorPositionMarkers_.markers.push_back(loco_ros::getInitializedMeasurementSphereMarker("odom", "measured_foot_position", i, markerScale));
      }

      // Planned and measured foot positions in Cartesian space.
      measuredEndEffectorPositionPub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("/loco_ros/measured_foot_positions", 1);

      ///////////////////////////////////////////////////////////////////////////////////////
      /// Contact forces.
      ///////////////////////////////////////////////////////////////////////////////////////

      // Publisher initialization: desired contact forces.
      desiredContactForceAtPositionPairs_.clear();
      measuredContactForceAtPositionPairs_.clear();
      for(auto& name : limbNames) {
        desiredContactForceAtPositionPairs_.push_back( std::make_pair( nodeHandle_.advertise<kindr_msgs::VectorAtPosition>(
            std::string("/loco_ros/desired_contact_force_") + name , 1), kindr_msgs::VectorAtPosition()));
        measuredContactForceAtPositionPairs_.push_back( std::make_pair( nodeHandle_.advertise<kindr_msgs::VectorAtPosition>(
            std::string("/loco_ros/measured_contact_force_") + name , 1), kindr_msgs::VectorAtPosition()));
      }

      // Message initialization.
      for (auto& pair : desiredContactForceAtPositionPairs_) {
        pair.second.header.frame_id = "odom";
        pair.second.position_frame_id = "odom";
        pair.second.type = kindr_msgs::VectorAtPosition::TYPE_FORCE;
      }
      for (auto& pair : measuredContactForceAtPositionPairs_) {
        pair.second.header.frame_id = "odom";
        pair.second.position_frame_id = "odom";
        pair.second.type = kindr_msgs::VectorAtPosition::TYPE_FORCE;
      }

      return true;
  }

  bool initialize(ros::NodeHandle& nodeHandle,
                  const loco::Limbs& limbs,
                  const double markerScale = 0.01,
                  const double forceVectorScale = 1.0)
  {
      std::vector<std::string> limbNames;
      for(const auto& limb: limbs) { limbNames.push_back(limb->getName()); }
      return initialize(nodeHandle, limbNames, markerScale, forceVectorScale);
  }

  bool shutdown() override {
    measuredEndEffectorPositionPub_.shutdown();

    for (auto& pair : desiredContactForceAtPositionPairs_) {
      pair.first.shutdown();
    }
    for (auto& pair : measuredContactForceAtPositionPairs_) {
      pair.first.shutdown();
    }
    return true;
  }

  bool update(const loco::Limbs& limbs) {
    const ros::Time timestamp = ros::Time::now();

    if (measuredEndEffectorPositionPub_.getNumSubscribers() > 0u) {
      for (const auto& limb : limbs) {
        const int limbId = limb->getId();
        const loco::Position& positionWorldToFootInWorldFrame = limb->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        measuredEndEffectorPositionMarkers_.markers[limbId].pose.position.x = positionWorldToFootInWorldFrame.x();
        measuredEndEffectorPositionMarkers_.markers[limbId].pose.position.y = positionWorldToFootInWorldFrame.y();
        measuredEndEffectorPositionMarkers_.markers[limbId].pose.position.z = positionWorldToFootInWorldFrame.z();
      }
    }

    // Contact forces
    unsigned int i = 0;
    for (auto& pair : desiredContactForceAtPositionPairs_) {
      if (pair.first.getNumSubscribers() > 0u) {
        const loco::Force& force = limbs.get(i).getEndEffector().getStateDesired().getForceAtEndEffectorInWorldFrame();
        const loco::Position& position = limbs.get(i).getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        pair.second.header.stamp = timestamp;

        pair.second.vector.x = force.x();
        pair.second.vector.y = force.y();
        pair.second.vector.z = force.z();

        pair.second.position.x = position.x();
        pair.second.position.y = position.y();
        pair.second.position.z = position.z();
      }
      ++i;
    }

    i = 0;
    for (auto& pair : measuredContactForceAtPositionPairs_) {
      if (pair.first.getNumSubscribers() > 0u) {
        const loco::Force& force = limbs.get(i).getEndEffector().getStateMeasured().getForceAtEndEffectorInWorldFrame();
        const loco::Position& position = limbs.get(i).getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        pair.second.header.stamp = timestamp;

        pair.second.vector.x = forceVectorScale_*force.x();
        pair.second.vector.y = forceVectorScale_*force.y();
        pair.second.vector.z = forceVectorScale_*force.z();

        pair.second.position.x = position.x();
        pair.second.position.y = position.y();
        pair.second.position.z = position.z();
      }
      ++i;
    }

    return true;
  }

  bool publish() override {
    loco_ros::publishMsg<visualization_msgs::MarkerArray>(measuredEndEffectorPositionPub_, measuredEndEffectorPositionMarkers_);

    // Contact forces
    for (auto& pair : desiredContactForceAtPositionPairs_) {
      loco_ros::publishMsg<kindr_msgs::VectorAtPosition>(pair.first, pair.second);
    }

    for (auto& pair : measuredContactForceAtPositionPairs_) {
      loco_ros::publishMsg<kindr_msgs::VectorAtPosition>(pair.first, pair.second);
    }

    return true;
  }


 protected:
  ros::NodeHandle nodeHandle_;
  double forceVectorScale_;

  ros::Publisher measuredEndEffectorPositionPub_;
  visualization_msgs::MarkerArray measuredEndEffectorPositionMarkers_;

  // Contact forces.
  std::list<std::pair<ros::Publisher, kindr_msgs::VectorAtPosition> > desiredContactForceAtPositionPairs_;
  std::list<std::pair<ros::Publisher, kindr_msgs::VectorAtPosition> > measuredContactForceAtPositionPairs_;
};


} /* namespace loco_ros */

