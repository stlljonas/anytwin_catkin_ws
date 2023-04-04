/*!
 * @file     AnymalJointStatesRosInitializationTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/initialization_traits/InitializationTraits.hpp"

// any msgs
#include <any_msgs/ExtendedJointState.h>

namespace anymal_model_ros {
namespace initialization_traits {

template <>
class InitializationTraits<any_msgs::ExtendedJointState> {
 public:
  using RosType = any_msgs::ExtendedJointState;

  static void initialize(RosType& ros);
};

using AnymalJointStatesRosInitializationTraits = InitializationTraits<any_msgs::ExtendedJointState>;

}  // namespace initialization_traits
}  // namespace anymal_model_ros
