/*
 * Point.cpp
 *
 *  Created on: Mar 1, 2017
 *      Author: Gabriel Hottiger, Dario Bellicoso
 */

#include "loco_ros/visualization/Point.hpp"

namespace loco_ros {

Point::Point() {

}

Point::~Point() {

}

bool Point::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  nodeHandle_ = nodeHandle;
  shutdown();
  markerArray_.markers.clear();
  markerArrayPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(topic, 1);
  return true;
}

void Point::shutdown() {
  markerArrayPublisher_.shutdown();
}

void Point::addPoint(const loco::Position& point, const double scale, const Color& color, const std::string ns) {
  visualization_msgs::Marker pointMarker =
      loco_ros::getInitializedSphereMarker("odom", ns, markerArray_.markers.size(),
                                           scale, color.rNormalized(), color.gNormalized(), color.bNormalized(), color.a());
  geometry_msgs::Point point_msg;
  point_msg.x = point.x();
  point_msg.y = point.y();
  point_msg.z = point.z();
  pointMarker.pose.position = point_msg;

  markerArray_.markers.push_back(pointMarker);
}

void Point::updatePoint(const loco::Position& point, unsigned int id) {
  if (markerArray_.markers.size() <= id) {
    addPoint(point, 1.0);
  } else {
    markerArray_.markers.at(id).pose.position.x = point.x();
    markerArray_.markers.at(id).pose.position.y = point.y();
    markerArray_.markers.at(id).pose.position.z = point.z();
  }
}

void Point::clearPoints() {
  markerArray_.markers.clear();
}

bool Point::visualize() {
  loco_ros::publishMsg(markerArrayPublisher_, markerArray_);
  return true;
}


} /* namespace loco_ros */
