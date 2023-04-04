/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Test transform listener implementation
 */

#include <gtest/gtest.h>

#include <tf2_ros/transform_broadcaster.h>

#include <tf/transform_listener.h>
#include <geometry_utils/geometry_utils.hpp>

#include "geometry_utils_ros/geometry_utils_ros.hpp"

namespace geometry_utils_ros {

class TestTransformListener : public ::testing::Test {
 public:
  TestTransformListener()
      : listener_{new TransformListenerRos()},
        broadcaster_{},
        transformOdomToWorld_{"world", "odom", Time(1.0),
                              geometry_utils::Transform(geometry_utils::Position(-1.0, -2.0, -3.0), geometry_utils::RotationQuaternion())} {
    tf::TransformListener wtfListener;  // TODO(ghottiger) Why the f*** do I need to put this here?????
    broadcaster_.sendTransform(toRos(transformOdomToWorld_));
  }
  std::unique_ptr<TransformListener> listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  geometry_utils::TransformStamped transformOdomToWorld_;
};

TEST_F(TestTransformListener, noTransformAvailable) {  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  geometry_utils::TransformStamped inexistingTransform;
  ASSERT_FALSE(listener_->getTransformation(&inexistingTransform, "world", "unexisting_frame", geometry_utils::Time(0.0)));
}

TEST_F(TestTransformListener, extrapolationNotAvailable) {  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  geometry_utils::TransformStamped transformOdomToWorld;
  ASSERT_FALSE(listener_->getTransformation(&transformOdomToWorld, "world", "odom", geometry_utils::Time(2.0)));
}

TEST_F(TestTransformListener, transformAvailable) {  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  geometry_utils::TransformStamped transformOdomToWorld;
  ASSERT_TRUE(listener_->getTransformation(&transformOdomToWorld, "world", "odom", geometry_utils::Time(1.0)));
}

TEST_F(TestTransformListener, inverseTransform) {  // NOLINT(modernize-use-equals-default, modernize-use-equals-delete)
  geometry_utils::TransformStamped transformWorldToOdom;
  ASSERT_TRUE(listener_->getTransformation(&transformWorldToOdom, "odom", "world", geometry_utils::Time(1.0)));
  ASSERT_EQ(transformWorldToOdom.frame_, std::string{"odom"});
  ASSERT_EQ(transformWorldToOdom.childFrame_, std::string{"world"});

  // Setup identity pose in odom
  geometry_utils::PoseStamped identity;
  identity.frame_ = "odom";

  // Transforms from source_frame/child_frame "odom" to target_frame/frame_id "world"
  geometry_utils::PoseStamped pose = identity;
  ASSERT_TRUE(pose.applyTransformation(transformOdomToWorld_));

  // Check transform
  ASSERT_EQ(pose.position_.x(), -1.0);
  ASSERT_EQ(pose.position_.y(), -2.0);
  ASSERT_EQ(pose.position_.z(), -3.0);
  ASSERT_EQ(pose.frame_, std::string{"world"});

  // Transforms from source_frame/child_frame "world" to target_frame/frame_id "odom"
  ASSERT_TRUE(listener_->transform(&pose, "odom", Time(1.0)));
  ASSERT_TRUE(pose.isEqual(identity));
}

}  // namespace geometry_utils_ros
