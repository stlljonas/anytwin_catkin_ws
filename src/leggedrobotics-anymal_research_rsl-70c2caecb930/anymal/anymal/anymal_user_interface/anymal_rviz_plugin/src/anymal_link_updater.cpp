/*
 * anymal_link_updater.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: Perry Franklin
 */

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "anymal_rviz_plugin/anymal_link_updater.hpp"

#include <anymal_model_ros/conversions.hpp>

namespace anymal_rviz_plugin {

AnymalLinkUpdater::AnymalLinkUpdater(const std::string& urdf_description, const urdf::Model& urdf_model) : urdf_model_(urdf_model) {
  model_.initializeFromUrdf(urdf_description);
}

AnymalLinkUpdater::~AnymalLinkUpdater() {}

void AnymalLinkUpdater::update(const anymal_msgs::AnymalState& state_msg) {
  anymal_model_ros::fromRos(state_msg, model_);
}

bool AnymalLinkUpdater::getLinkTransforms(const std::string& link_name, Ogre::Vector3& visual_position,
                                          Ogre::Quaternion& visual_orientation, Ogre::Vector3& collision_position,
                                          Ogre::Quaternion& collision_orientation) const {
  bool success = false;

  kindr::HomTransformQuatD transform_in_anymal_world;
  if (getLinkTransform(link_name, transform_in_anymal_world)) {
    Ogre::Vector3 ogre_position(transform_in_anymal_world.getPosition().x(), transform_in_anymal_world.getPosition().y(),
                                transform_in_anymal_world.getPosition().z());

    Ogre::Quaternion ogre_quaternion(transform_in_anymal_world.getRotation().w(), transform_in_anymal_world.getRotation().x(),
                                     transform_in_anymal_world.getRotation().y(), transform_in_anymal_world.getRotation().z());

    visual_position = ogre_position;
    visual_orientation = ogre_quaternion;
    collision_position = ogre_position;
    collision_orientation = ogre_quaternion;

    success = true;

  } else {
    success = false;
    ROS_ERROR_STREAM("AnymalLinkUpdater failed to find a transform for link [" << link_name << "]");
  }

  return success;
}

const anymal_model::AnymalModel& AnymalLinkUpdater::getModel() const {
  return model_;
}

kindr::HomTransformQuatD urdfPoseToKindrHomTransform(const urdf::Pose& urdf_pose) {
  kindr::HomTransformQuatD out;

  out.getPosition().x() = urdf_pose.position.x;
  out.getPosition().y() = urdf_pose.position.y;
  out.getPosition().z() = urdf_pose.position.z;

  out.getRotation().x() = urdf_pose.rotation.x;
  out.getRotation().y() = urdf_pose.rotation.y;
  out.getRotation().z() = urdf_pose.rotation.z;
  out.getRotation().w() = urdf_pose.rotation.w;

  return out;
}

bool AnymalLinkUpdater::getLinkTransform(const std::string& link_name, kindr::HomTransformQuatD& transform) const {
  bool success = false;

  // Two success cases:
  // The link is in the AnymalModel -> use that transform
  // The link is a child of a fixed joint in the URDF, in which case we use parent link to find this link's transform
  bool link_is_in_anymal_model = false;
  anymal_model::AnymalModel::BodyEnum body_enum;

  try {
    body_enum = anymal_description::AnymalTopology::bodyKeys.atName(link_name).getEnum();
    link_is_in_anymal_model = true;
  } catch (const std::out_of_range& ex) {
    link_is_in_anymal_model = false;
  }

  // If we found the link in the AnymalTopology...
  if (link_is_in_anymal_model) {
    // Get the transform from the model
    transform.getPosition() =
        kindr::Position3D(model_.getPositionWorldToBody(body_enum, anymal_model::AnymalModel::CoordinateFrameEnum::WORLD));
    const Eigen::Matrix3d& orientation = model_.getOrientationWorldToBody(body_enum);

    // The orientation is the inverse of what we need
    transform.getRotation() = kindr::RotationMatrixD(orientation).transpose();

    success = true;

  } else {  // If the link is not in the Anymal

    urdf::LinkConstSharedPtr urdf_link = urdf_model_.getLink(link_name);

    // If the link exists in the urdf model...
    if (urdf_link) {
      urdf::JointConstSharedPtr urdf_joint = urdf_link->parent_joint;

      // If the joint is fixed...
      if (urdf_joint->type == urdf::Joint::FIXED) {
        // We try to get the transform of the parent link, and apply our transform to that.
        kindr::HomTransformQuatD parent_link_transform;
        ;
        const std::string& parent_link_name = urdf_joint->parent_link_name;

        if (getLinkTransform(parent_link_name, parent_link_transform)) {
          kindr::HomTransformQuatD parent_to_joint = urdfPoseToKindrHomTransform(urdf_joint->parent_to_joint_origin_transform);

          transform = parent_link_transform * parent_to_joint;

          success = true;

        } else {
          // When the parent transform fails, we fail.
          success = false;
        }

      } else {
        // When the joint is not fixed, we won't try to guess where it is and just fail.
        success = false;
      }

    } else {
      // We did not find the link in the urdf model
      success = false;
    }
  }

  return success;
}

}  // namespace anymal_rviz_plugin
