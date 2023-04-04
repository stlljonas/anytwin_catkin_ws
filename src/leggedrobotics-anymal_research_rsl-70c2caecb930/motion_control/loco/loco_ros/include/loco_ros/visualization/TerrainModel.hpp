/*
 * TerrainModel.hpp
 *
 *  Created on: Jan 19, 2017
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include <loco/common/TerrainModelBase.hpp>

// ros
#include <ros/ros.h>

// msgs
#include <visualization_msgs/MarkerArray.h>

namespace loco_ros {

class TerrainModel {
 public:
  TerrainModel()
      : nodeHandle_(),
        arrowLength_(),
        terrainModelMarkers_(),
        terrainModelPub_()
  {

  }

  virtual ~TerrainModel() { }

  bool initialize(ros::NodeHandle& nodeHandle,
                  const double planeSize = 1.0,
                  const double arrowLength = 0.1) {
    nodeHandle_ = nodeHandle;
    arrowLength_ = arrowLength;

    // Terrain model markers.
    terrainModelMarkers_.markers.clear();
    visualization_msgs::Marker planeMarker = getInitializedPlaneMarker("odom", "terrain_model", 0, planeSize, planeSize, 0.0001);
    planeMarker.color.r = 0.0;
    planeMarker.color.g = 0.8;
    planeMarker.color.b = 0.0;
    planeMarker.color.a = 0.5;
    terrainModelMarkers_.markers.push_back(planeMarker);

    terrainModelMarkers_.markers.push_back(getInitializedReferenceArrowMarker("odom", "terrain_model", 1, 0.01));
    terrainModelMarkers_.markers[1].points.clear();
    terrainModelMarkers_.markers[1].points.push_back(geometry_msgs::Point());
    terrainModelMarkers_.markers[1].points.push_back(geometry_msgs::Point());

    // Terrain model.
    terrainModelPub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("/loco_ros/terrain_model", 1);

    return true;
  }

  bool shutdown() {
    terrainModelPub_.shutdown();
    return true;
  }

  bool update(const loco::WholeBody& wholeBody, const loco::TerrainModelBase& terrainModel) {
    if (terrainModelPub_.getNumSubscribers() > 0u) {
      const loco::Position& positionWorldToBaseInWorldFrame = wholeBody.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
      const loco::RotationQuaternion& orientationWorldToControl = wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();

      const loco::Position positionWorldToBaseInWorldFrameOnPlane =
          terrainModel.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(positionWorldToBaseInWorldFrame);

      terrainModelMarkers_.markers[0].pose.position.x = positionWorldToBaseInWorldFrameOnPlane.x();
      terrainModelMarkers_.markers[0].pose.position.y = positionWorldToBaseInWorldFrameOnPlane.y();
      terrainModelMarkers_.markers[0].pose.position.z = positionWorldToBaseInWorldFrameOnPlane.z();

      terrainModelMarkers_.markers[0].pose.orientation.w = orientationWorldToControl.w();
      terrainModelMarkers_.markers[0].pose.orientation.x = -orientationWorldToControl.x();
      terrainModelMarkers_.markers[0].pose.orientation.y = -orientationWorldToControl.y();
      terrainModelMarkers_.markers[0].pose.orientation.z = -orientationWorldToControl.z();

      terrainModelMarkers_.markers[1].points[0] = terrainModelMarkers_.markers[0].pose.position;

      loco::Vector normalToPlane;
      terrainModel.getNormal(positionWorldToBaseInWorldFrame, normalToPlane);

      const loco::Position arrowStart{
        terrainModelMarkers_.markers[0].pose.position.x,
        terrainModelMarkers_.markers[0].pose.position.y,
        terrainModelMarkers_.markers[0].pose.position.z};

      const loco::Position arrowEnd{
        arrowStart.x() + arrowLength_*normalToPlane.x(),
        arrowStart.y() + arrowLength_*normalToPlane.y(),
        arrowStart.z() + arrowLength_*normalToPlane.z()};

      terrainModelMarkers_.markers[1].points[1].x = arrowEnd.x();
      terrainModelMarkers_.markers[1].points[1].y = arrowEnd.y();
      terrainModelMarkers_.markers[1].points[1].z = arrowEnd.z();
    }

    return true;
  }

  bool publish() {
    publishMsg(terrainModelPub_, terrainModelMarkers_);
    return true;
  }

protected:
  ros::NodeHandle nodeHandle_;
  double arrowLength_;
  visualization_msgs::MarkerArray terrainModelMarkers_;
  ros::Publisher terrainModelPub_;


};


} /* namespace loco_ros */

