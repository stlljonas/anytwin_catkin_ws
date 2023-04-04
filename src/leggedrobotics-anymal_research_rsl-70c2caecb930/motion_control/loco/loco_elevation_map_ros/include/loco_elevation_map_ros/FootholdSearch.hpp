/*
 * FootholdSearch.hpp
 *
 *  Created on: May 27, 2017
 *      Author: Tanja Baumann
 */

#pragma once

// ros
#include <ros/ros.h>

// msgs
#include <visualization_msgs/MarkerArray.h>

// loco
#include <loco/common/legs/Legs.hpp>

// loco_ros
#include <loco_ros/loco_ros.hpp>

// loco elevation map
#include <loco_elevation_map/TerrainModelElevationMap.hpp>

// grid map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include "grid_map_core/grid_map_core.hpp"

namespace loco_ros {

class FootholdSearch {
 public:
  FootholdSearch()
      : numLegs_(4),
        invertedPendulumFootholdPub_(),
        zeroOneFootholdPub_(),
        polygonFootholdPub_(),
        searchRadiusPub_(),
        invertedPendulumFootholdMarkers_(),
        zeroOneFootholdMarkers_(),
        polygonFootholdMarkers_(),
        searchRadiusMarkers_(),
        polygonLineMarkers_(),
        gridMapMsg_() {
    searchRadiusMarkers_.markers.resize(4, visualization_msgs::Marker());
    polygonLineMarkers_.markers.resize(4, visualization_msgs::Marker());
  }
  ;

  virtual ~FootholdSearch() {

  }
  ;

  bool initialize(ros::NodeHandle& nodeHandle, bool isSimulation = false) {
    nodeHandle_ = nodeHandle;

    invertedPendulumFootholdMarkers_.markers.clear();
    for (int k = 0; k < numLegs_; k++) {
      invertedPendulumFootholdMarkers_.markers.push_back(
          getInitializedReferenceSphereMarker("odom", "inverted_pendulum_foothold", k, 0.03, 0, 0, 1));
    }
    zeroOneFootholdMarkers_.markers.clear();
    for (int k = 0; k < numLegs_; k++) {
      zeroOneFootholdMarkers_.markers.push_back(
          getInitializedReferenceSphereMarker("odom", "zero_one_foothold", k, 0.01, 0, 1, 0));
    }
    polygonFootholdMarkers_.markers.clear();
    for (int k = 0; k < numLegs_; k++) {
      polygonFootholdMarkers_.markers.push_back(
          getInitializedReferenceSphereMarker("odom", "polygon_foothold", k, 0.03, 1, 0, 1));
    }

    invertedPendulumFootholdPub_ = nodeHandle_.advertise < visualization_msgs::MarkerArray
        > ("/loco_ros/inverted_pendulum_footholds", 1);
    zeroOneFootholdPub_ = nodeHandle_.advertise < visualization_msgs::MarkerArray > ("/loco_ros/zero_one_footholds", 1);
    polygonFootholdPub_ = nodeHandle_.advertise < visualization_msgs::MarkerArray > ("/loco_ros/polygon_footholds", 1);

    searchRadiusPub_ = nodeHandle.advertise < visualization_msgs::MarkerArray > ("/loco_ros/foothold_search_radius", 1);
    polygonLinePub_ = nodeHandle.advertise < visualization_msgs::MarkerArray > ("/loco_ros/foothold_polygon_line", 1);
    gridMapPub_ = nodeHandle.advertise < grid_map_msgs::GridMap > ("/loco_ros/polygon_gridmap", 1);

    int i = 0;
    for (auto& marker : searchRadiusMarkers_.markers) {
      marker.pose.orientation.w = 1.0;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.lifetime = ros::Duration(0.0);
      marker.scale.x = 0.005;
      marker.scale.y = 0.005;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.id = 2;
      marker.header.frame_id = "odom";
      marker.ns = std::string { "pos_" } + std::to_string(i);
      i++;
    }

    i = 0;
    for (auto& marker : polygonLineMarkers_.markers) {
      marker.pose.orientation.w = 1.0;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.lifetime = ros::Duration(0.0);
      marker.scale.x = 0.005;
      marker.scale.y = 0.005;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.id = 2;
      marker.header.frame_id = "odom";
      marker.ns = std::string { "pos_" } + std::to_string(i);
      i++;
    }
    return true;
  }

  bool shutdown() {
    invertedPendulumFootholdPub_.shutdown();
    zeroOneFootholdPub_.shutdown();
    polygonFootholdPub_.shutdown();
    searchRadiusPub_.shutdown();
    polygonLinePub_.shutdown();
    gridMapPub_.shutdown();
    return true;
  }

  bool update(const loco::Legs& legs, const loco::TerrainModelElevationMap& terrain) {
    const ros::Time timestamp = ros::Time::now();

    for (const auto& leg : legs) {
      int legId = leg->getId();
      loco::Position invertedPendulumFoothold;
      loco::Position zeroOneFoothold;
      loco::Position polygonFoothold;
      double searchRadius;
      double lineLength;
      loco::LinearVelocity commandVelocity;
      grid_map::GridMap map;

      terrain.getFootholdsForVisualization(legId, invertedPendulumFoothold, zeroOneFoothold, polygonFoothold,
                                           searchRadius, lineLength, commandVelocity, map);

      invertedPendulumFootholdMarkers_.markers[legId].pose.position.x = invertedPendulumFoothold.x();
      invertedPendulumFootholdMarkers_.markers[legId].pose.position.y = invertedPendulumFoothold.y();
      invertedPendulumFootholdMarkers_.markers[legId].pose.position.z = invertedPendulumFoothold.z();
      invertedPendulumFootholdMarkers_.markers[legId].header.stamp = timestamp;

      zeroOneFootholdMarkers_.markers[legId].pose.position.x = zeroOneFoothold.x();
      zeroOneFootholdMarkers_.markers[legId].pose.position.y = zeroOneFoothold.y();
      zeroOneFootholdMarkers_.markers[legId].pose.position.z = zeroOneFoothold.z();
      zeroOneFootholdMarkers_.markers[legId].header.stamp = timestamp;

      polygonFootholdMarkers_.markers[legId].pose.position.x = polygonFoothold.x();
      polygonFootholdMarkers_.markers[legId].pose.position.y = polygonFoothold.y();
      polygonFootholdMarkers_.markers[legId].pose.position.z = polygonFoothold.z();
      polygonFootholdMarkers_.markers[legId].header.stamp = timestamp;

      searchRadiusMarkers_.markers[legId].header.stamp = timestamp;
      geometry_msgs::Point point;
      searchRadiusMarkers_.markers[legId].points.clear();
      int nPoints = 20;
      const double pi = std::acos(-1);
      double t = 0;
      double dt = 2 * pi / nPoints;
      for (int i = 0; i < nPoints + 1; i++) {
        point.x = invertedPendulumFoothold.x() + searchRadius * std::cos(t);
        point.y = invertedPendulumFoothold.y() + searchRadius * std::sin(t);
        point.z = invertedPendulumFoothold.z();
        searchRadiusMarkers_.markers[legId].points.push_back(point);
        t += dt;
      }

      //Line polygon visualization
      polygonLineMarkers_.markers[legId].header.stamp = timestamp;
      polygonLineMarkers_.markers[legId].points.clear();
      point.x = polygonFoothold.x();
      point.y = polygonFoothold.y();
      point.z = polygonFoothold.z();
      polygonLineMarkers_.markers[legId].points.push_back(point);
      if (commandVelocity.vector().norm() != 0) {
        commandVelocity.vector().normalize();
      }
      point.x = polygonFoothold.x() + commandVelocity.x() * lineLength;
      point.y = polygonFoothold.y() + commandVelocity.y() * lineLength;
      point.z = polygonFoothold.z();
      polygonLineMarkers_.markers[legId].points.push_back(point);

      grid_map::GridMapRosConverter::toMessage(map, gridMapMsg_);

    }

    return true;
  }

  bool publish() {
    publishMsg<visualization_msgs::MarkerArray>(invertedPendulumFootholdPub_, invertedPendulumFootholdMarkers_);
    publishMsg<visualization_msgs::MarkerArray>(zeroOneFootholdPub_, zeroOneFootholdMarkers_);
    publishMsg<visualization_msgs::MarkerArray>(polygonFootholdPub_, polygonFootholdMarkers_);
    publishMsg<visualization_msgs::MarkerArray>(searchRadiusPub_, searchRadiusMarkers_);
    publishMsg<visualization_msgs::MarkerArray>(polygonLinePub_, polygonLineMarkers_);
    publishMsg<grid_map_msgs::GridMap>(gridMapPub_, gridMapMsg_);
    return true;
  }
 protected:

  unsigned int numLegs_;
  ros::NodeHandle nodeHandle_;

  ros::Publisher invertedPendulumFootholdPub_;
  ros::Publisher zeroOneFootholdPub_;
  ros::Publisher polygonFootholdPub_;
  ros::Publisher searchRadiusPub_;
  ros::Publisher polygonLinePub_;
  ros::Publisher gridMapPub_;

  visualization_msgs::MarkerArray invertedPendulumFootholdMarkers_;
  visualization_msgs::MarkerArray zeroOneFootholdMarkers_;
  visualization_msgs::MarkerArray polygonFootholdMarkers_;
  visualization_msgs::MarkerArray searchRadiusMarkers_;
  visualization_msgs::MarkerArray polygonLineMarkers_;
  grid_map_msgs::GridMap gridMapMsg_;

};

} /* namespace loco_ros */

