/*
 * SupportPolygonSequence.cpp
 *
 *  Created on: Feb 1, 2017
 *      Author: dbellicoso
 */


// anymal ctrl dynamic gaits ros
#include "motion_generation_ros/SupportPolygonSequence.hpp"

namespace anymal_ctrl_dynamic_gaits_ros {

SupportPolygonSequence::SupportPolygonSequence()
  :  nodeHandle_(),
     supportPolygonSequence_(),
     publisher_(),
     colorId_(0u)
{

}

SupportPolygonSequence::~SupportPolygonSequence() {

}

void SupportPolygonSequence::setColorVector(std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>>& colors) {
  colors_ = colors;
}

void SupportPolygonSequence::setColorVectorId(unsigned int colorId) {
  colorId_ = colorId;
}

bool SupportPolygonSequence::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  nodeHandle_ = nodeHandle;
  publisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(topic, 100);

  return true;
}

bool SupportPolygonSequence::shutdown() {
  publisher_.shutdown();
  return true;
}

bool SupportPolygonSequence::update(const zmp::MotionPlan& motionPlan) {
  if (!motionPlan.didOptimizationSucceeded()) {
    return true;
  }

  if (getNumSubscribers() > 0u) {
    supportPolygonSequence_.markers.clear();

    unsigned int id = 0u;
    unsigned int offset = 0u;

    for (const auto& polygon : motionPlan.getSupportPolygonsInPlaneFrame()) {
      // There are no support polygons for full flight phases
      if (polygon.getPolygonType() == zmp::PolygonType::Empty ) {
        ++offset;
        continue;
      }

      visualization_msgs::Marker supportPolygonMarker;
      supportPolygonMarker.pose.orientation.w = 1.0;
      supportPolygonMarker.action = visualization_msgs::Marker::ADD;
      supportPolygonMarker.type = visualization_msgs::Marker::LINE_STRIP;
      supportPolygonMarker.scale.x = 0.00200;
      supportPolygonMarker.scale.y = 0.00200;
      supportPolygonMarker.scale.z = 0.00200;
      supportPolygonMarker.color.a = 1.0;
      supportPolygonMarker.header.frame_id = "odom";

      const unsigned int colorId = robot_utils::intmod(id+offset+colorId_, colors_.size());

      supportPolygonMarker.color.r = colors_[colorId].x();
      supportPolygonMarker.color.g = colors_[colorId].y();
      supportPolygonMarker.color.b = colors_[colorId].z();

      supportPolygonMarker.id = id;
      supportPolygonMarker.ns = "support_polygon_" + std::to_string(supportPolygonMarker.id);

      geometry_msgs::Point point;
      supportPolygonMarker.points.reserve(polygon.getPolygon().getVertices().size());
      for (const auto& vertex : polygon.getPolygon().getVertices()) {
        const loco::Pose& posePlaneToWorld = motionPlan.getVirtualPlaneFrame().getPosePlaneToWorld();
        const loco::Position positionWorldToVertexInWorldFrame =
            posePlaneToWorld.transform(loco::Position(vertex.x(), vertex.y(), 0.0));
        point.x = positionWorldToVertexInWorldFrame.x();
        point.y = positionWorldToVertexInWorldFrame.y();
        point.z = positionWorldToVertexInWorldFrame.z();
        supportPolygonMarker.points.push_back(point);
      }

      // Add first point again to reconnect the polygon.
      if (supportPolygonMarker.points.size() > 2) {
        supportPolygonMarker.points.push_back(supportPolygonMarker.points.front());
        supportPolygonSequence_.markers.push_back(supportPolygonMarker);
      }

      ++id;
    }
  }

  return true;
}

bool SupportPolygonSequence::publish() {
  loco_ros::publishMsg(publisher_, supportPolygonSequence_);
  return true;
}

unsigned int SupportPolygonSequence::getNumSubscribers() const {
  return publisher_.getNumSubscribers();
}

} /* namespace loco */
