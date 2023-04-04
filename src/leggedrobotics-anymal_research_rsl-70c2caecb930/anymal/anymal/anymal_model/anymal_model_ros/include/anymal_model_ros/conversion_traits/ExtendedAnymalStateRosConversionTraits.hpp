/*!
 * @file     ExtendedAnymalStateRosConversionTraits.hpp
 * @author   Markus Staeuble
 * @date     Mar, 2018
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/conversion_traits/ConversionTraits.hpp"

// anymal model
#include <anymal_model/ExtendedAnymalState.hpp>

// anymal msgs
#include <anymal_msgs/AnymalState.h>

namespace anymal_model_ros {
namespace conversion_traits {

template <>
class ConversionTraits<anymal_model::ExtendedAnymalState, anymal_msgs::AnymalState> {
 public:
  using ObjectType = anymal_model::ExtendedAnymalState;
  using RosType = anymal_msgs::AnymalState;

  // Calling this function requires an initialized ROS message.
  static void convert(const ObjectType& object, RosType& ros);
  static RosType convert(const ObjectType& object);

  static void convert(const RosType& ros, ObjectType& object);
  static ObjectType convert(const RosType& ros);
};

using ExtendedAnymalStateRosConversionTraits = ConversionTraits<anymal_model::ExtendedAnymalState, anymal_msgs::AnymalState>;

}  // namespace conversion_traits
}  // namespace anymal_model_ros
