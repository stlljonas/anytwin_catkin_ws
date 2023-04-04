/*!
 * @file     AnymalJointStatesRosConversionTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/conversion_traits/ConversionTraits.hpp"

// anymal model
#include <anymal_model/AnymalJointState.hpp>

// any msgs
#include <any_msgs/ExtendedJointState.h>

namespace anymal_model_ros {
namespace conversion_traits {

template <>
class ConversionTraits<anymal_model::AnymalJointState, any_msgs::ExtendedJointState> {
 public:
  using ObjectType = anymal_model::AnymalJointState;
  using RosType = any_msgs::ExtendedJointState;

  static void convert(const ObjectType& object, RosType& ros);
  static RosType convert(const ObjectType& object);

  static void convert(const RosType& ros, ObjectType& object);
  static ObjectType convert(const RosType& ros);
};

using AnymalJointStatesRosConversionTraits = ConversionTraits<anymal_model::AnymalJointState, any_msgs::ExtendedJointState>;

}  // namespace conversion_traits
}  // namespace anymal_model_ros
