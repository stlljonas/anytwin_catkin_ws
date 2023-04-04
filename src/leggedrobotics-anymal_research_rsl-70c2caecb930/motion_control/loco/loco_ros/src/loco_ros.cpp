/*
 * loco_ros.cpp
 *
 *  Created on: Apr 5, 2016
 *      Author: dbellicoso, Christian Gehring
 */

#include "loco_ros/loco_ros.hpp"

namespace loco_ros {

void initializeVectorAtPositionMessage(kindr_msgs::VectorAtPosition& msg) {
  msg.header.frame_id = "odom";
}

visualization_msgs::Marker getInitializedMarker(unsigned int markerType,
                                                const std::string& frame_id,
                                                const std::string& ns,
                                                const int id,
                                                const double scale,
                                                const float red,
                                                const float green,
                                                const float blue,
                                                const float a) {
  visualization_msgs::Marker marker;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = markerType;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.a = a;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.id = id;
  marker.ns = ns;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  return marker;
}

visualization_msgs::Marker getInitializedCubeMarker(const std::string& frame_id,
                                                    const std::string& ns,
                                                    const int id,
                                                    const double scale,
                                                    const float red,
                                                    const float green,
                                                    const float blue,
                                                    const float a)
{
  return getInitializedMarker(visualization_msgs::Marker::CUBE, frame_id, ns, id, scale, red, green, blue, a);
}

visualization_msgs::Marker getInitializedPlaneMarker(const std::string& frame_id,
                                                     const std::string& ns,
                                                     const int id,
                                                     const double lenX,
                                                     const double lenY,
                                                     const double lenZ)
{
  visualization_msgs::Marker marker = getInitializedCubeMarker(frame_id, ns, id);
  marker.scale.x = lenX;
  marker.scale.y = lenY;
  marker.scale.z = lenZ;
  return marker;
}

visualization_msgs::Marker getInitializedSphereMarker(const std::string& frame_id,
                                                      const std::string& ns,
                                                      const int id,
                                                      const double scale,
                                                      const float red,
                                                      const float green,
                                                      const float blue,
                                                      const float a)
{
  return getInitializedMarker(visualization_msgs::Marker::SPHERE, frame_id, ns, id, scale, red, green, blue, a);
}

visualization_msgs::Marker getInitializedReferenceSphereMarker(const std::string& frame_id,
                                                               const std::string& ns,
                                                               const int id,
                                                               const double scale,
                                                               const float red,
                                                               const float green,
                                                               const float blue,
                                                               const float a)
{
  return getInitializedSphereMarker(frame_id, ns, id, scale, red, green, blue, a);
}

visualization_msgs::Marker getInitializedMeasurementSphereMarker(const std::string& frame_id,
                                                                 const std::string& ns,
                                                                 const int id,
                                                                 const double scale,
                                                                 const float red,
                                                                 const float green,
                                                                 const float blue,
                                                                 const float a)
{
  return getInitializedSphereMarker(frame_id, ns, id, scale, red, green, blue, a);
}

visualization_msgs::Marker getInitializedReferenceArrowMarker(const std::string& frame_id,
                                                              const std::string& ns,
                                                              const int id,
                                                              const double scale,
                                                              const float red,
                                                              const float green,
                                                              const float blue,
                                                              const float a)
{
  visualization_msgs::Marker marker = getInitializedMarker(visualization_msgs::Marker::ARROW,
                                                           frame_id, ns, id, scale, red, green, blue, a);
  marker.scale.x = scale;
  marker.scale.y = 2.4*scale;
  marker.scale.z = 1.4*scale;
  return marker;
}

visualization_msgs::Marker getInitializedMeasurementArrowMarker(const std::string& frame_id,
                                                                const std::string& ns,
                                                                const int id,
                                                                const double scale,
                                                                const float red,
                                                                const float green,
                                                                const float blue,
                                                                const float a)
{
  return getInitializedReferenceArrowMarker(frame_id, ns, id, scale, red, green, blue, a);
}

}
