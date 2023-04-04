/*
 * WholeBodyDynamics.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Dario Bellicoso
 */

// loco ros
#include <loco_ros_anymal/visualization/WholeBodyDynamics.hpp>

namespace loco_ros_anymal {

bool WholeBodyDynamics::initialize(ros::NodeHandle& nodeHandle) {
  nodeHandle_ = nodeHandle;

  centerOfMass_.first = nodeHandle_.advertise<decltype(centerOfMass_.second)>("/loco_ros/dynamics/center_of_mass", 1);
  inertiaEllipsoids_.first = nodeHandle_.advertise<decltype(inertiaEllipsoids_.second)>("/loco_ros/dynamics/inertia_ellipsoid", 1);

  // (Number of legs) * (number of links per leg) + 1(base)
//    const unsigned int numBodies = wholeBody.getLegs().size()*wholeBody.getLegs().get(0).getLinks().size() + 1;
  constexpr unsigned int numBodies = 13 + 4;

  visualization_msgs::Marker marker;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.id = 0;
  marker.ns = "";
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "base";
  centerOfMass_.second = marker;

  centerOfMass_.second.points = std::vector<geometry_msgs::Point>(numBodies, geometry_msgs::Point());

  for (unsigned int k=0; k<numBodies; ++k) {
    inertiaEllipsoids_.second.markers.push_back(loco_ros::getInitializedReferenceSphereMarker("base", "inertia_ellipsoids_markers", 0, 0.3));
  }

  return true;
}

bool WholeBodyDynamics::shutdown() {
  centerOfMass_.first.shutdown();
  inertiaEllipsoids_.first.shutdown();

  return true;
}

bool WholeBodyDynamics::update(const loco::WholeBody& wholeBody) {
  if (centerOfMass_.first.getNumSubscribers() > 0u) {
    const ros::Time timestamp = ros::Time::now();

    centerOfMass_.second.header.stamp = timestamp;

    const loco::Position& positionBaseToBaseComInBaseFrame = wholeBody.getTorso().getProperties().getBaseToCenterOfMassPositionInBaseFrame();
    centerOfMass_.second.points[0].x = positionBaseToBaseComInBaseFrame.x();
    centerOfMass_.second.points[0].y = positionBaseToBaseComInBaseFrame.y();
    centerOfMass_.second.points[0].z = positionBaseToBaseComInBaseFrame.z();

    for (unsigned int k=1; k<13; k++) {
      const unsigned int bodiesPerLeg = wholeBody.getLegs().get(0).getLinks().size();
      const unsigned int legId = (k-1)/bodiesPerLeg;
      const unsigned int bodyId = (k-1) - legId*bodiesPerLeg;

      const loco::Position& positionBaseToBodyCoMInBaseFrame = wholeBody.getLegs().get(legId).getLinks().get(bodyId).getBaseToCoMPositionInBaseFrame();

      centerOfMass_.second.points[k].x = positionBaseToBodyCoMInBaseFrame.x();
      centerOfMass_.second.points[k].y = positionBaseToBodyCoMInBaseFrame.y();
      centerOfMass_.second.points[k].z = positionBaseToBodyCoMInBaseFrame.z();
    }

    for (unsigned int k=13; k<17; k++) {
      const unsigned int legId = k-13;
      const loco::Position& positionBaseToLegCoMInBaseFrame = wholeBody.getLegs().get(legId).getLegProperties().getPositionBaseToLimbComInBaseFrame();
      centerOfMass_.second.points[k].x = positionBaseToLegCoMInBaseFrame.x();
      centerOfMass_.second.points[k].y = positionBaseToLegCoMInBaseFrame.y();
      centerOfMass_.second.points[k].z = positionBaseToLegCoMInBaseFrame.z();
    }

  }

  if (inertiaEllipsoids_.first.getNumSubscribers() > 0u) {
    // todo: update inertia ellipsoids
  }

  return true;
}

bool WholeBodyDynamics::publish() {
  loco_ros::publishMsg(centerOfMass_);
  return true;
}


} /* namespace loco_ros */

