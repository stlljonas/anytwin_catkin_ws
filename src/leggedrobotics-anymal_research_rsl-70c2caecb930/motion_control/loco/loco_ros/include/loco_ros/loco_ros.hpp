/*
 * loco_ros_utils.hpp
 *
 *  Created on: Nov 6, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

#include <ros/ros.h>
#include <kindr_msgs/VectorAtPosition.h>
#include <visualization_msgs/MarkerArray.h>


namespace loco_ros {

/*! Publish a ROS message.
 *
 * \note This function is deprecated as ROS already does this check:
 * http://docs.ros.org/indigo/api/roscpp/html/topic__manager_8cpp_source.html#l00710
 * Also see https://git.anybotics.com/anybotics/anybotics/merge_requests/1808
 *
 */
template<typename MsgType_>
void publishMsg(const ros::Publisher& publisher, const MsgType_& msg) {

  using MsgTypePtr = boost::shared_ptr< MsgType_>;
  using MsgTypeConstPtr = boost::shared_ptr< MsgType_ const>;

  if (publisher.getNumSubscribers() > 0 || publisher.isLatched()) {
    MsgTypePtr msgPtr(new MsgType_(msg));
    publisher.publish(MsgTypeConstPtr(msgPtr));
  }
}

/*! Publish a ROS message.
 *
 * \note This function is deprecated as ROS already checks for this:
 * http://docs.ros.org/indigo/api/roscpp/html/topic__manager_8cpp_source.html#l00710
 * Also see https://git.anybotics.com/anybotics/anybotics/merge_requests/1808
 *
 */
template<typename MsgType_>
void publishMsg(const std::pair<ros::Publisher, MsgType_>& pair) {

  using MsgTypePtr = boost::shared_ptr<MsgType_>;
  using MsgTypeConstPtr = boost::shared_ptr< MsgType_ const>;

  if (pair.first.getNumSubscribers() > 0 || pair.first.isLatched()) {
    MsgTypePtr msgPtr(new MsgType_(pair.second));
    pair.first.publish(MsgTypeConstPtr(msgPtr));
  }
}

void initializeVectorAtPositionMessage(kindr_msgs::VectorAtPosition& msg);

visualization_msgs::Marker getInitializedMarker(unsigned int markerType,
                                                const std::string& frame_id,
                                                const std::string& ns = "",
                                                int id = 0,
                                                double scale = 0.01,
                                                float red = 1.0f,
                                                float green = 0.0f,
                                                float blue = 0.0f,
                                                float a = 1.0f);

visualization_msgs::Marker getInitializedCubeMarker(const std::string& frame_id,
                                                    const std::string& ns = "",
                                                    int id = 0,
                                                    double scale = 0.01,
                                                    float red = 1.0f,
                                                    float green = 0.0f,
                                                    float blue = 0.0f,
                                                    float a = 1.0f);

visualization_msgs::Marker getInitializedPlaneMarker(const std::string& frame_id,
                                                     const std::string& ns = "",
                                                     int id = 0,
                                                     double lenX = 1.0,
                                                     double lenY = 1.0,
                                                     double lenZ = 1.0);

visualization_msgs::Marker getInitializedSphereMarker(const std::string& frame_id,
                                                      const std::string& ns = "",
                                                      int id = 0,
                                                      double scale = 0.01,
                                                      float red = 1.0f,
                                                      float green = 0.0f,
                                                      float blue = 0.0f,
                                                      float a = 1.0f);

visualization_msgs::Marker getInitializedReferenceSphereMarker(const std::string& frame_id,
                                                               const std::string& ns = "",
                                                               int id = 0,
                                                               double scale = 0.01,
                                                               float red = 1.0f,
                                                               float green = 0.0f,
                                                               float blue = 0.0f,
                                                               float a = 1.0f);

visualization_msgs::Marker getInitializedMeasurementSphereMarker(const std::string& frame_id,
                                                                 const std::string& ns = "",
                                                                 int id = 0,
                                                                 double scale = 0.01,
                                                                 float red = 0.0f,
                                                                 float green = 0.0f,
                                                                 float blue = 1.0f,
                                                                 float a = 1.0f);

visualization_msgs::Marker getInitializedReferenceArrowMarker(const std::string& frame_id,
                                                              const std::string& ns = "",
                                                              int id = 0,
                                                              double scale = 0.01,
                                                              float red = 1.0f,
                                                              float green = 0.0f,
                                                              float blue = 0.0f,
                                                              float a = 1.0f);

visualization_msgs::Marker getInitializedMeasurementArrowMarker(const std::string& frame_id,
                                                                const std::string& ns = "",
                                                                int id = 0,
                                                                double scale = 0.01,
                                                                float red = 0.0f,
                                                                float green = 0.0f,
                                                                float blue = 1.0f,
                                                                float a = 1.0f);

} // namespace
