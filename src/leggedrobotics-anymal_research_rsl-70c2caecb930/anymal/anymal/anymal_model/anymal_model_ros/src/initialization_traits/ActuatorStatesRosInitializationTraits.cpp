/*!
 * @file     ActuatorStatesRosInitializationTraits.cpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

// anymal model ros
#include "anymal_model_ros/initialization_traits/ActuatorStatesRosInitializationTraits.hpp"
#include "anymal_model_ros/typedefs.hpp"

namespace anymal_model_ros {
namespace initialization_traits {

void ActuatorStatesRosInitializationTraits::initialize(RosType& ros) {
  constexpr auto numActuators = AD::getActuatorsDimension();
  ros.states.resize(numActuators);
  for (const auto& key : AD::getActuatorKeys()) {
    const auto actuatorId = key.getId();
    ros.states[actuatorId].name = key.getName();
    ros.states[actuatorId].header.frame_id = key.getName();
  }
}

}  // namespace initialization_traits
}  // namespace anymal_model_ros
