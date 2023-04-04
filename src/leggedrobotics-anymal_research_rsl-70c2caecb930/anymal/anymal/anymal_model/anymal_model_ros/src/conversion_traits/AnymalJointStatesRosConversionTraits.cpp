/*!
 * @file     AnymalJointStatesRosConversionTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

// anymal model ros
#include "anymal_model_ros/conversion_traits/AnymalJointStatesRosConversionTraits.hpp"
#include "anymal_model_ros/typedefs.hpp"

// any measurements ros
#include <any_measurements_ros/any_measurements_ros.hpp>

namespace anymal_model_ros {
namespace conversion_traits {

void AnymalJointStatesRosConversionTraits::convert(const ObjectType& object, RosType& ros) {
  for (auto jointKey : AD::getJointKeys()) {
    const auto jointEnum = jointKey.getEnum();
    const auto jointId = jointKey.getId();
    ros.position[jointId] = object[jointEnum].position_;
    ros.velocity[jointId] = object[jointEnum].velocity_;
    ros.acceleration[jointId] = object[jointEnum].acceleration_;
    ros.effort[jointId] = object[jointEnum].effort_;
  }
  any_measurements_ros::toRos(object[CAD::ConcreteTopology::JointEnum::LF_HAA].time_, ros.header.stamp);
}

AnymalJointStatesRosConversionTraits::RosType AnymalJointStatesRosConversionTraits::convert(const ObjectType& object) {
  RosType ros;
  convert(object, ros);
  return ros;
}

void AnymalJointStatesRosConversionTraits::convert(const RosType& ros, ObjectType& object) {
  for (auto jointKey : AD::getJointKeys()) {
    const auto jointEnum = jointKey.getEnum();
    const auto jointId = jointKey.getId();
    any_measurements_ros::fromRos(ros.header.stamp, object[jointEnum].time_);
    object[jointEnum].position_ = ros.position[jointId];
    object[jointEnum].velocity_ = ros.velocity[jointId];
    object[jointEnum].acceleration_ = ros.acceleration[jointId];
    object[jointEnum].effort_ = ros.effort[jointId];
  }
}

AnymalJointStatesRosConversionTraits::ObjectType AnymalJointStatesRosConversionTraits::convert(const RosType& ros) {
  ObjectType object;
  convert(ros, object);
  return object;
}

}  // namespace conversion_traits
}  // namespace anymal_model_ros
