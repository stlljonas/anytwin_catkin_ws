/*
 * SwingTrajectoryGeneratorSpline.cpp
 *
 *  Created on: Jun 4, 2016
 *      Author: Christian Gehring, Dario Bellicoso
 */

// loco ros
#include "loco_ros/visualization/SwingTrajectoryGeneratorSpline.hpp"
#include <loco_ros/loco_ros.hpp>

namespace loco_ros {


SwingTrajectoryGeneratorSpline::SwingTrajectoryGeneratorSpline()
{
  markerArray_.markers.resize(4, visualization_msgs::Marker());
}

SwingTrajectoryGeneratorSpline::~SwingTrajectoryGeneratorSpline() {

}

bool SwingTrajectoryGeneratorSpline::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  publisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(topic, 1);

  int i=0;
  for (auto& marker : markerArray_.markers) {
    marker.pose.orientation.w = 1.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(0.0);
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.id = i;
    marker.header.frame_id = "odom";
    marker.ns = std::string{"pos_"} + std::to_string(i);
    ++i;
  }

  return true;
}

bool SwingTrajectoryGeneratorSpline::shutdown() {
  publisher_.shutdown();
  return true;
}

bool SwingTrajectoryGeneratorSpline::update(
    const loco::SwingTrajectoryGeneratorSpline& generator,
    const loco::TorsoBase& torso) {
  if (publisher_.getNumSubscribers() > 0u) {
    const ros::Time stamp = ros::Time::now();

    int legId = 0;
    for (auto& marker : markerArray_.markers) {
      marker.header.stamp = stamp;

      const auto& splineX = generator.getSplineX()[legId];
      const auto& splineY = generator.getSplineY()[legId];
      const auto& splineZ = generator.getSplineZ()[legId];

      constexpr int nPoints = 20;
      marker.points.clear();
      marker.points.reserve(nPoints);

      const double minTime = splineX.getMinTime();
      const double maxTime = splineX.getMaxTime();

      double time = minTime;
      const double dt = (maxTime - minTime) / (nPoints - 1);

      geometry_msgs::Point point;
      for (int i = 0; i < nPoints; ++i) {
        //Ensure that time is never out of bounds (numerical errors)
        if (time > maxTime) {
          time = maxTime;
        }
        splineX.evaluate(point.x, time);
        splineY.evaluate(point.y, time);
        splineZ.evaluate(point.z, time);
        marker.points.push_back(point);

        time += dt;
      }
      ++legId;
    }

  }
  return true;
}

bool SwingTrajectoryGeneratorSpline::visualize() {
  loco_ros::publishMsg(publisher_, markerArray_);
  return true;
}

} /* namespace loco_ros */
