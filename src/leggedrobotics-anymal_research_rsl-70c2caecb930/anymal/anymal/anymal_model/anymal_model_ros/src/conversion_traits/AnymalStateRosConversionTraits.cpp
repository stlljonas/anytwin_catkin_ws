/*!
 * @file     AnymalStateRosConversionTraits.cpp
 * @author   Peter Fankhauser, Dario Bellicoso
 * @date     Dec, 2016
 */

// anymal model ros
#include "anymal_model_ros/conversion_traits/AnymalStateRosConversionTraits.hpp"
#include "anymal_model_ros/typedefs.hpp"

// kindr ros
#include <kindr_ros/kindr_ros.hpp>

namespace anymal_model_ros {
namespace conversion_traits {

void AnymalStateRosConversionTraits::convert(const ObjectType& object, RosType& ros) {
  // Joints.
  for (unsigned int i = 0; i < anymal_model::AnymalState::getNumberOfJointPositions(); ++i) {
    ros.joints.position[i] = object.getJointPositions()(i);
    ros.joints.velocity[i] = object.getJointVelocities()(i);
    ros.joints.acceleration[i] = object.getJointAccelerations()(i);
    ros.joints.effort[i] = object.getJointTorques()(i);
  }

  // Base position.
  kindr_ros::convertToRosGeometryMsg(object.getPositionWorldToBaseInWorldFrame(), ros.pose.pose.position);

  // Base orientation.
  kindr_ros::convertToRosGeometryMsg(object.getOrientationBaseToWorld(), ros.pose.pose.orientation);

  // Base linear velocity.
  kindr_ros::convertToRosGeometryMsg(object.getOrientationBaseToWorld().inverseRotate(object.getLinearVelocityBaseInWorldFrame()),
                                     ros.twist.twist.linear);

  // Base angular velocity.
  kindr_ros::convertToRosGeometryMsg(object.getAngularVelocityBaseInBaseFrame(), ros.twist.twist.angular);

  // Frame transforms. Frame names need to be initialized by the concrete
  // robot description, otherwise some (slow) string operations will be
  // necessary
  for (unsigned int i = 0; i < ros.frame_transforms.size(); ++i) {
    kindr_ros::convertToRosGeometryMsg(object.getFrameTransform(static_cast<AT::FrameTransformEnum>(i)), ros.frame_transforms[i].transform);
  }
}

AnymalStateRosConversionTraits::RosType AnymalStateRosConversionTraits::convert(const ObjectType& object) {
  RosType ros;
  convert(object, ros);
  return ros;
}

void AnymalStateRosConversionTraits::convert(const RosType& ros, ObjectType& object) {
  // Joints.
  for (size_t i = 0; i < ros.joints.position.size(); ++i) {
    object.getJointPositions()(i) = ros.joints.position[i];
    object.getJointVelocities()(i) = ros.joints.velocity[i];
    object.getJointAccelerations()(i) = ros.joints.acceleration[i];
    object.getJointTorques()(i) = ros.joints.effort[i];
  }

  // Base position.
  anymal_model::Position basePosition;
  kindr_ros::convertFromRosGeometryMsg(ros.pose.pose.position, basePosition);
  object.setPositionWorldToBaseInWorldFrame(basePosition);

  // Base orientation.
  anymal_model::RotationQuaternion orientationBaseToWorld;
  kindr_ros::convertFromRosGeometryMsg(ros.pose.pose.orientation, orientationBaseToWorld);
  object.setOrientationBaseToWorld(orientationBaseToWorld);

  // Base linear velocity.
  anymal_model::LinearVelocity baseLinearVelocityInBase;
  kindr_ros::convertFromRosGeometryMsg(ros.twist.twist.linear, baseLinearVelocityInBase);
  const anymal_model::LinearVelocity baseLinearVelocityInWorld = orientationBaseToWorld.rotate(baseLinearVelocityInBase);
  object.setLinearVelocityBaseInWorldFrame(baseLinearVelocityInWorld);

  // Base angular velocity.
  anymal_model::LocalAngularVelocity localAngularVelocity;
  kindr_ros::convertFromRosGeometryMsg(ros.twist.twist.angular, localAngularVelocity);
  object.setAngularVelocityBaseInBaseFrame(localAngularVelocity);

  // Frame transforms
  for (const auto& tf : ros.frame_transforms) {
    const std::string transformName = tf.child_frame_id + "_to_" + tf.header.frame_id;
    anymal_model::Pose transform;
    kindr_ros::convertFromRosGeometryMsg(tf.transform, transform);

    constexpr auto frameKeys = AD::getKeys<AT::FrameTransformEnum>();
    object.setFrameTransform(frameKeys.atName(transformName).getEnum(), transform);
  }
}

AnymalStateRosConversionTraits::ObjectType AnymalStateRosConversionTraits::convert(const RosType& ros) {
  ObjectType object;
  convert(ros, object);
  return object;
}

}  // namespace conversion_traits
}  // namespace anymal_model_ros
