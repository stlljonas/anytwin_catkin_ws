/*!
 * @file     AnymalContactsRosInitializationTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/initialization_traits/InitializationTraits.hpp"

// anymal msgs
#include <anymal_msgs/Contact.h>

// stl
#include <vector>

namespace anymal_model_ros {
namespace initialization_traits {

template <>
class InitializationTraits<std::vector<anymal_msgs::Contact>> {
 public:
  using RosType = std::vector<anymal_msgs::Contact>;

  static void initialize(RosType& ros);
};

using AnymalContactsRosInitializationTraits = InitializationTraits<std::vector<anymal_msgs::Contact>>;

}  // namespace initialization_traits
}  // namespace anymal_model_ros
