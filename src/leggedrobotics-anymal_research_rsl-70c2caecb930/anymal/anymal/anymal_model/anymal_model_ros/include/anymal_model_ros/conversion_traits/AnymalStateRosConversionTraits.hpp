/*!
 * @file     AnymalStateRosConversionTraits.hpp
 * @author   Peter Fankhauser, Dario Bellicoso
 * @date     Dec, 2016
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/conversion_traits/ConversionTraits.hpp"

// anymal model
#include <anymal_model/AnymalState.hpp>

// anymal msgs
#include <anymal_msgs/AnymalState.h>

namespace anymal_model_ros {
namespace conversion_traits {

template <>
class ConversionTraits<anymal_model::AnymalState, anymal_msgs::AnymalState> {
 public:
  using ObjectType = anymal_model::AnymalState;
  using RosType = anymal_msgs::AnymalState;

  static void convert(const ObjectType& object, RosType& ros);
  static RosType convert(const ObjectType& object);

  static void convert(const RosType& ros, ObjectType& object);
  static ObjectType convert(const RosType& ros);
};

using AnymalStateRosConversionTraits = ConversionTraits<anymal_model::AnymalState, anymal_msgs::AnymalState>;

}  // namespace conversion_traits
}  // namespace anymal_model_ros
