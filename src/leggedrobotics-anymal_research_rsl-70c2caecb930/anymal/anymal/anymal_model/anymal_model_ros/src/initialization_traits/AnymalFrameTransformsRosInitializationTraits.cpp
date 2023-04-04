/*!
 * @file     AnymalFrameTransformsRosInitializationTraits.cpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

// anymal model ros
#include "anymal_model_ros/initialization_traits/AnymalFrameTransformsRosInitializationTraits.hpp"

namespace anymal_model_ros {
namespace initialization_traits {

void AnymalFrameTransformsRosInitializationTraits::initialize(RosType& ros) {
  ros.clear();

  geometry_msgs::TransformStamped transform;
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;

  transform.header.frame_id = "odom";
  transform.child_frame_id = "footprint";
  ros.push_back(transform);

  transform.header.frame_id = "odom";
  transform.child_frame_id = "feetcenter";
  ros.push_back(transform);
}

}  // namespace initialization_traits
}  // namespace anymal_model_ros
