/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Tests geometry_utils_ros conversions
 */

#include <gtest/gtest.h>

#include <geometry_utils/geometry_utils.hpp>

#include "geometry_utils_ros/geometry_utils_ros.hpp"

namespace geometry_utils_ros {

TEST(TestConversion, poseStamped) {  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  geometry_utils::PoseStamped pose{"test", geometry_utils::Time(3.1415), geometry_utils::Position(0.2, 1.9, -0.3),
                                   geometry_utils::RotationQuaternion(geometry_utils::EulerAnglesZyx(0.2, 0.1, 0.3))};
  auto rosMsg = geometry_utils_ros::toRos(pose);
  auto convertedPose = geometry_utils_ros::fromRos(rosMsg);
  ASSERT_TRUE(convertedPose.isEqual(pose));
  ASSERT_TRUE(convertedPose.hasEqualStamp(pose));
}

TEST(TestConversion, twistStamped) {  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  geometry_utils::TwistStamped twist{"test", geometry_utils::Time(3.1415), geometry_utils::LinearVelocity(0.2, 1.9, -0.3),
                                     geometry_utils::LocalAngularVelocity(0.2, 0.1, 0.3)};
  auto rosMsg = geometry_utils_ros::toRos(twist);
  auto convertedTwist = geometry_utils_ros::fromRos(rosMsg);
  ASSERT_TRUE(convertedTwist.isEqual(twist));
  ASSERT_TRUE(convertedTwist.hasEqualStamp(twist));
}

TEST(TestConversion, transformStamped) {  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  geometry_utils::TransformStamped transform{
      "test", "test2", geometry_utils::Time(3.1415),
      geometry_utils::Transform(geometry_utils::Position(0.2, 1.9, -0.3),
                                geometry_utils::RotationQuaternion(geometry_utils::EulerAnglesZyx(0.2, 0.1, 0.3)))};
  auto rosMsg = geometry_utils_ros::toRos(transform);
  auto convertedTransform = geometry_utils_ros::fromRos(rosMsg);
  ASSERT_TRUE(convertedTransform.isEqual(transform));
  ASSERT_TRUE(convertedTransform.hasEqualStamp(transform));
}

}  // namespace geometry_utils_ros
