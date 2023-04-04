/*
 * Arrow.cpp
 *
 *  Created on: Apr 5, 2016
 *      Author: Dario Bellicoso
 */

#include "loco_ros/visualization/Arrow.hpp"

#include "loco_ros/loco_ros.hpp"

namespace loco_ros {

Arrow::Arrow() {

}

Arrow::~Arrow() {

}

void Arrow::setColorVector(std::vector<loco::Vector>& colors) {
	colors_ = colors;
}

bool Arrow::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  nodeHandle_ = nodeHandle;

  shutdown();

  markerArray_.markers.clear();
  markerArrayPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(topic, 1);
  return true;
}

void Arrow::shutdown() {
  markerArrayPublisher_.shutdown();
}

void Arrow::addArrow(const loco::Position& arrowStart, const loco::Position& arrowEnd, unsigned int id) {
  visualization_msgs::Marker arrowMarker = loco_ros::getInitializedReferenceArrowMarker("odom", "arrows", markerArray_.markers.size());
  geometry_msgs::Point point;

  point.x = arrowStart.x();
  point.y = arrowStart.y();
  point.z = arrowStart.z();
  arrowMarker.points.push_back(point);

  point.x = arrowEnd.x();
  point.y = arrowEnd.y();
  point.z = arrowEnd.z();
  arrowMarker.points.push_back(point);

  if (colors_.size()>0) {
		arrowMarker.color.r = colors_[id].x();
		arrowMarker.color.g = colors_[id].y();
		arrowMarker.color.b = colors_[id].z();
		arrowMarker.ns = "arrown_" + std::to_string(id);
  }

  markerArray_.markers.push_back(arrowMarker);
}

void Arrow::addArrow(const loco::Position& arrowStart, const loco::Vector& arrowComponents, unsigned int id) {
  visualization_msgs::Marker arrowMarker = loco_ros::getInitializedReferenceArrowMarker("odom", "arrows", markerArray_.markers.size());
  geometry_msgs::Point point;

  point.x = arrowStart.x();
  point.y = arrowStart.y();
  point.z = arrowStart.z();
  arrowMarker.points.push_back(point);

  point.x = arrowMarker.points.back().x + arrowComponents.x();
  point.y = arrowMarker.points.back().y + arrowComponents.y();
  point.z = arrowMarker.points.back().z + arrowComponents.z();
  arrowMarker.points.push_back(point);

  if (colors_.size()>0) {
  	arrowMarker.color.r = colors_[id].x();
		arrowMarker.color.g = colors_[id].y();
		arrowMarker.color.b = colors_[id].z();
		//arrowMarker.ns = "arrown_" + std::to_string(id);
  }

  markerArray_.markers.push_back(arrowMarker);
}

void Arrow::updateArrow(const loco::Position& arrowStart, const loco::Position& arrowEnd, unsigned int id) {
  if (markerArray_.markers.size() <= id) {
    addArrow(arrowStart, arrowEnd);
  } else {
    markerArray_.markers.at(id).points[0].x = arrowStart.x();
    markerArray_.markers.at(id).points[0].y = arrowStart.y();
    markerArray_.markers.at(id).points[0].z = arrowStart.z();

    markerArray_.markers.at(id).points[1].x = arrowEnd.x();
    markerArray_.markers.at(id).points[1].y = arrowEnd.y();
    markerArray_.markers.at(id).points[1].z = arrowEnd.z();
  }
}

void Arrow::updateArrow(const loco::Position& arrowStart, const loco::Vector& arrowComponents, unsigned int id) {
  if (markerArray_.markers.size() <= id) {
    addArrow(arrowStart, arrowComponents);
  } else {
    markerArray_.markers.at(id).points[0].x = arrowStart.x();
    markerArray_.markers.at(id).points[0].y = arrowStart.y();
    markerArray_.markers.at(id).points[0].z = arrowStart.z();

    markerArray_.markers.at(id).points[1].x = markerArray_.markers.at(id).points[0].x + arrowComponents.x();
    markerArray_.markers.at(id).points[1].y = markerArray_.markers.at(id).points[0].y + arrowComponents.y();
    markerArray_.markers.at(id).points[1].z = markerArray_.markers.at(id).points[0].z + arrowComponents.z();
  }
}

void Arrow::clearArrows() {
  markerArray_.markers.clear();
}

bool Arrow::visualize() {
  loco_ros::publishMsg(markerArrayPublisher_, markerArray_);
  return true;
}


} /* namespace loco_ros */
