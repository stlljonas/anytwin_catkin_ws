/*!
 * @file     AnymalJointStatesRosInitializationTraits.cpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

// anymal model ros
#include "anymal_model_ros/initialization_traits/AnymalJointStatesRosInitializationTraits.hpp"
#include "anymal_model_ros/typedefs.hpp"

namespace anymal_model_ros {
namespace initialization_traits {

void AnymalJointStatesRosInitializationTraits::initialize(RosType& ros) {
  constexpr auto numJoints = AD::getJointsDimension();
  ros.position.assign(numJoints, 0.0);
  ros.velocity.assign(numJoints, 0.0);
  ros.acceleration.assign(numJoints, 0.0);
  ros.effort.assign(numJoints, 0.0);
  ros.name.clear();
  ros.name.reserve(numJoints);
  for (const auto& key : AD::getJointKeys()) {
    ros.name.emplace_back(key.getName());
  }
}

}  // namespace initialization_traits
}  // namespace anymal_model_ros
