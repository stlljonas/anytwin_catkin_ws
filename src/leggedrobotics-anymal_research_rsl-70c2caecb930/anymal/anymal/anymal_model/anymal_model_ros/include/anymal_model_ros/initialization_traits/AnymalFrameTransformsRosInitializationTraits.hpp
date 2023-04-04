/*!
 * @file     AnymalFrameTransformsRosInitializationTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/initialization_traits/InitializationTraits.hpp"

// geometry msgs
#include <geometry_msgs/TransformStamped.h>

// stl
#include <vector>

namespace anymal_model_ros {
namespace initialization_traits {

template <>
class InitializationTraits<std::vector<geometry_msgs::TransformStamped>> {
 public:
  using RosType = std::vector<geometry_msgs::TransformStamped>;

  static void initialize(RosType& ros);
};

using AnymalFrameTransformsRosInitializationTraits = InitializationTraits<std::vector<geometry_msgs::TransformStamped>>;

}  // namespace initialization_traits
}  // namespace anymal_model_ros
