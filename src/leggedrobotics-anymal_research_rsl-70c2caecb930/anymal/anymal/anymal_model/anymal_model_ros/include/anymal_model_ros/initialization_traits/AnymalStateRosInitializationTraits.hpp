/*!
 * @file     AnymalStateRosInitializationTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/initialization_traits/InitializationTraits.hpp"

// anymal msgs
#include <anymal_msgs/AnymalState.h>

namespace anymal_model_ros {
namespace initialization_traits {

template <>
class InitializationTraits<anymal_msgs::AnymalState> {
 public:
  using RosType = anymal_msgs::AnymalState;

  static void initialize(RosType& ros);
};

using AnymalStateRosInitializationTraits = InitializationTraits<anymal_msgs::AnymalState>;

}  // namespace initialization_traits
}  // namespace anymal_model_ros
