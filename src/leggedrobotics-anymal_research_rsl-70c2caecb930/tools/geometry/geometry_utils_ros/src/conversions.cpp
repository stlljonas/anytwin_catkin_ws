/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Ros conversions for geometry_utils common classes.
 */

#include "geometry_utils_ros/conversions.hpp"

#include <any_measurements_ros/any_measurements_ros.hpp>
#include <kindr_ros/kindr_ros.hpp>

namespace geometry_utils_ros {

geometry_utils::PoseStamped fromRos(const geometry_msgs::PoseStamped& poseStampedMsg) {
  geometry_utils::PoseStamped poseStamped;
  poseStamped.frame_ = poseStampedMsg.header.frame_id;
  poseStamped.stamp_ = any_measurements_ros::fromRos(poseStampedMsg.header.stamp);
  kindr_ros::convertFromRosGeometryMsg(poseStampedMsg.pose.position, poseStamped.position_);
  kindr_ros::convertFromRosGeometryMsg(poseStampedMsg.pose.orientation, poseStamped.orientation_);
  return poseStamped;
}

geometry_utils::TwistStamped fromRos(const geometry_msgs::TwistStamped& twistStampedMsg) {
  geometry_utils::TwistStamped twistStamped;
  twistStamped.frame_ = twistStampedMsg.header.frame_id;
  twistStamped.stamp_ = any_measurements_ros::fromRos(twistStampedMsg.header.stamp);
  kindr_ros::convertFromRosGeometryMsg(twistStampedMsg.twist.linear, twistStamped.linearVelocity_);
  kindr_ros::convertFromRosGeometryMsg(twistStampedMsg.twist.angular, twistStamped.angularVelocity_);
  return twistStamped;
}

geometry_utils::TransformStamped fromRos(const geometry_msgs::TransformStamped& transformStampedMsg) {
  geometry_utils::TransformStamped transformStamped;
  transformStamped.frame_ = transformStampedMsg.header.frame_id;
  transformStamped.childFrame_ = transformStampedMsg.child_frame_id;
  transformStamped.stamp_ = any_measurements_ros::fromRos(transformStampedMsg.header.stamp);
  kindr_ros::convertFromRosGeometryMsg(transformStampedMsg.transform, transformStamped.transform_);
  return transformStamped;
}

geometry_msgs::PoseStamped toRos(const geometry_utils::PoseStamped& poseStamped) {
  geometry_msgs::PoseStamped poseStampedMsg;
  poseStampedMsg.header.frame_id = poseStamped.frame_;
  poseStampedMsg.header.stamp = any_measurements_ros::toRos(poseStamped.stamp_);
  kindr_ros::convertToRosGeometryMsg(poseStamped.position_, poseStampedMsg.pose.position);
  kindr_ros::convertToRosGeometryMsg(poseStamped.orientation_, poseStampedMsg.pose.orientation);
  return poseStampedMsg;
}

geometry_msgs::TwistStamped toRos(const geometry_utils::TwistStamped& twistStamped) {
  geometry_msgs::TwistStamped twistStampedMsg;
  twistStampedMsg.header.frame_id = twistStamped.frame_;
  twistStampedMsg.header.stamp = any_measurements_ros::toRos(twistStamped.stamp_);
  kindr_ros::convertToRosGeometryMsg(twistStamped.linearVelocity_, twistStampedMsg.twist.linear);
  kindr_ros::convertToRosGeometryMsg(twistStamped.angularVelocity_, twistStampedMsg.twist.angular);
  return twistStampedMsg;
}

geometry_msgs::TransformStamped toRos(const geometry_utils::TransformStamped& transformStamped) {
  geometry_msgs::TransformStamped transformStampedMsg;
  transformStampedMsg.header.frame_id = transformStamped.frame_;
  transformStampedMsg.child_frame_id = transformStamped.childFrame_;
  transformStampedMsg.header.stamp = any_measurements_ros::toRos(transformStamped.stamp_);
  kindr_ros::convertToRosGeometryMsg(transformStamped.transform_, transformStampedMsg.transform);
  return transformStampedMsg;
}

}  // namespace geometry_utils_ros
