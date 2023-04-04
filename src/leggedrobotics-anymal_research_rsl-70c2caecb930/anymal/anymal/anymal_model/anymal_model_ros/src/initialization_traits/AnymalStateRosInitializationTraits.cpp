/*!
 * @file     AnymalStateRosInitializationTraits.cpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

// anymal model ros
#include "anymal_model_ros/initialization_traits/AnymalStateRosInitializationTraits.hpp"
#include "anymal_model_ros/initializations.hpp"

namespace anymal_model_ros {
namespace initialization_traits {

void AnymalStateRosInitializationTraits::initialize(RosType& ros) {
  ros.state = anymal_msgs::AnymalState::STATE_ERROR_UNKNOWN;
  ros.header.frame_id = "odom";
  ros.pose.header.frame_id = "odom";
  ros.pose.pose.orientation.w = 1.0;
  ros.pose.pose.orientation.x = 0.0;
  ros.pose.pose.orientation.y = 0.0;
  ros.pose.pose.orientation.z = 0.0;
  ros.pose.pose.position.x = 0.0;
  ros.pose.pose.position.y = 0.0;
  ros.pose.pose.position.z = 0.0;
  ros.twist.header.frame_id = "base";
  anymal_model_ros::initialize(ros.contacts);
  anymal_model_ros::initialize(ros.joints);
  anymal_model_ros::initialize(ros.frame_transforms);
}

}  // namespace initialization_traits
}  // namespace anymal_model_ros
