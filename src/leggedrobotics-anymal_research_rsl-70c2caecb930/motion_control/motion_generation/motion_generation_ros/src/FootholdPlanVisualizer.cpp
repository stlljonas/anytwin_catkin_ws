/*
 * FootholdPlanVisualizer.cpp
 *
 *  Created on: Feb 1, 2017
 *      Author: dbellicoso
 */


// anymal ctrl dynamic gaits ros
#include "motion_generation_ros/FootholdPlanVisualizer.hpp"

namespace anymal_ctrl_dynamic_gaits_ros {

FootholdPlanVisualizer::FootholdPlanVisualizer()
  :  nodeHandle_(),
     footholdBounds_(),
     collisionBounds_()
{
  publisherRefs_.push_back(footholdBounds_.first);
  publisherRefs_.push_back(collisionBounds_.first);
}

void FootholdPlanVisualizer::setColorVector(const std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>>& colors) {
  colors_ = colors;
}

bool FootholdPlanVisualizer::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  nodeHandle_ = nodeHandle;
  footholdBounds_.first = nodeHandle.advertise<decltype(footholdBounds_.second)>(topic + std::string{"_foothold_bounds"}, 1);
  collisionBounds_.first = nodeHandle.advertise<decltype(collisionBounds_.second)>(topic + std::string{"_collision_bounds"}, 1);
  return true;
}

bool FootholdPlanVisualizer::update(const loco::foothold_generator::FootholdPlan& plan) {
  if (isNotSubscribed()) {
    return true;
  }

  footholdBounds_.second.markers.clear();
  collisionBounds_.second.markers.clear();

  // Plot bounded polygon constraints
  unsigned int id = 0;
  for (const auto& polygon : plan.getBoundedPolygonsInPlaneFrame()) {
    if (polygon.getNumVertices() > 0u) {
      visualization_msgs::Marker polygonMarker;
      polygonMarker.pose.orientation.w = 1.0;
      polygonMarker.action = visualization_msgs::Marker::ADD;
      polygonMarker.type = visualization_msgs::Marker::LINE_STRIP;
      polygonMarker.scale.x = 0.002;
      polygonMarker.scale.y = 0.002;
      polygonMarker.scale.z = 0.002;
      polygonMarker.color.a = 1.0;
      polygonMarker.header.frame_id = "odom";

      polygonMarker.color.r = colors_[id+1].x();
      polygonMarker.color.g = colors_[id+1].y();
      polygonMarker.color.b = colors_[id+1].z();
      polygonMarker.id = id++;
      polygonMarker.ns = "bound_" + std::to_string(polygonMarker.id);

      for (const auto& vertex : polygon.getVertices()) {
        const loco::Position positionWorldToVertexInWorldFrame =
            plan.getVirtualPlaneFrame().getPosePlaneToWorld().transform(
                loco::Position(vertex.x(), vertex.y(), 0.0));

        geometry_msgs::Point point;
        point.x = positionWorldToVertexInWorldFrame.x();
        point.y = positionWorldToVertexInWorldFrame.y();
        point.z = positionWorldToVertexInWorldFrame.z();
        polygonMarker.points.push_back(point);
      }

      // Add first point again to reconnect the polygon.
      if (polygonMarker.points.size() > 2) {
        polygonMarker.points.push_back(polygonMarker.points.front());
      }
      footholdBounds_.second.markers.push_back(polygonMarker);
    }
  }

  // Plot collision line constraints
  for (unsigned int legId=0; legId<plan.getCollisionLinesInPlaneFrame().size(); ++legId) {
    id = 0;
    for (unsigned int legIdNeighbour=0; legIdNeighbour<plan.getCollisionLinesInPlaneFrame()[legId].size(); ++legIdNeighbour) {

      // leg cannot be its own neighbor
      if (legId == legIdNeighbour) {
        continue;
      }

      for (const auto& line : plan.getCollisionLinesInPlaneFrame()[legId][legIdNeighbour]) {
        if (line.getNumVertices() > 0u) {
           visualization_msgs::Marker polygonMarker;
           polygonMarker.pose.orientation.w = 1.0;
           polygonMarker.action = visualization_msgs::Marker::ADD;
           polygonMarker.type = visualization_msgs::Marker::LINE_STRIP;
           polygonMarker.scale.x = 0.001;
           polygonMarker.scale.y = 0.001;
           polygonMarker.color.a = 1.0;
           polygonMarker.header.frame_id = "odom";

           polygonMarker.color.r = colors_[legId+1].x();
           polygonMarker.color.g = colors_[legId+1].y();
           polygonMarker.color.b = colors_[legId+1].z();
           polygonMarker.id = id++;
           polygonMarker.ns = "collision_lines_" + std::to_string(legId);

           polygonMarker.points.reserve(line.getVertices().size());
           for (const auto& vertex : line.getVertices()) {
             const loco::Position positionWorldToVertexInWorldFrame =
                 plan.getVirtualPlaneFrame().getPosePlaneToWorld().transform(
                     loco::Position(vertex.x(), vertex.y(), 0.0));

             geometry_msgs::Point point;
             point.x = positionWorldToVertexInWorldFrame.x();
             point.y = positionWorldToVertexInWorldFrame.y();
             point.z = positionWorldToVertexInWorldFrame.z();
             polygonMarker.points.push_back(point);
           }
           collisionBounds_.second.markers.push_back(polygonMarker);
        }
      } // end for lineId
    } // end for legIdNeighbor
  } // end for legId

  return true;
}

bool FootholdPlanVisualizer::publish() {
  loco_ros::publishMsg(footholdBounds_);
  loco_ros::publishMsg(collisionBounds_);
  return true;
}

} /* namespace loco */
