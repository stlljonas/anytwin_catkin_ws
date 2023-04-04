/*!
 * @file     ExtendedAnymalStateRosConversionTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

// anymal model ros
#include "anymal_model_ros/conversion_traits/ExtendedAnymalStateRosConversionTraits.hpp"
#include "anymal_model_ros/conversions.hpp"
#include "anymal_model_ros/initializations.hpp"
#include "anymal_model_ros/typedefs.hpp"

// any measurements ros
#include <any_measurements_ros/any_measurements_ros.hpp>

// kindr ros
#include <kindr_ros/kindr_ros.hpp>

namespace anymal_model_ros {
namespace conversion_traits {

void ExtendedAnymalStateRosConversionTraits::convert(const ObjectType& object, RosType& ros) {
  toRos(object.anymalState_, ros);

  ros::Time timeMsg = any_measurements_ros::toRos(object.time_);
  ros.header.stamp = timeMsg;
  ros.joints.header.stamp = timeMsg;
  ros.pose.header.stamp = timeMsg;
  ros.twist.header.stamp = timeMsg;

  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto contactId = contactKey.getId();
    auto& contactRos = ros.contacts[contactId];
    const auto& contactShm = object.contacts_[contactEnum];
    contactRos.header.stamp = timeMsg;
    kindr_ros::convertToRosGeometryMsg(contactShm.normal_, contactRos.normal);
    kindr_ros::convertToRosGeometryMsg(contactShm.position_, contactRos.position);
    any_measurements_ros::toRos(contactShm.wrench_, contactRos.wrench);
    contactRos.state = static_cast<int8_t>(contactShm.state_);
  }

  for (auto& tf : ros.frame_transforms) {
    tf.header.stamp = timeMsg;
  }

  ros.state = static_cast<int8_t>(object.status_);
}

ExtendedAnymalStateRosConversionTraits::RosType ExtendedAnymalStateRosConversionTraits::convert(const ObjectType& object) {
  RosType ros;
  initialize(ros);
  convert(object, ros);
  return ros;
}

void ExtendedAnymalStateRosConversionTraits::convert(const RosType& ros, ObjectType& object) {
  object.time_ = any_measurements_ros::fromRos(ros.header.stamp);

  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto contactId = contactKey.getId();
    auto& contactShm = object.contacts_[contactEnum];
    const auto& contactRos = ros.contacts[contactId];
    contactShm.time_ = any_measurements_ros::fromRos(contactRos.header.stamp);
    kindr_ros::convertFromRosGeometryMsg(contactRos.normal, contactShm.normal_);
    kindr_ros::convertFromRosGeometryMsg(contactRos.position, contactShm.position_);
    contactShm.wrench_ = any_measurements_ros::fromRos(contactRos.wrench);
    contactShm.state_ = static_cast<int>(contactRos.state);
  }

  anymal_model_ros::fromRos(ros, object.anymalState_);
  object.status_ = static_cast<anymal_model::StateStatus>(ros.state);
}

ExtendedAnymalStateRosConversionTraits::ObjectType ExtendedAnymalStateRosConversionTraits::convert(const RosType& ros) {
  ObjectType object;
  convert(ros, object);
  return object;
}

}  // namespace conversion_traits
}  // namespace anymal_model_ros
