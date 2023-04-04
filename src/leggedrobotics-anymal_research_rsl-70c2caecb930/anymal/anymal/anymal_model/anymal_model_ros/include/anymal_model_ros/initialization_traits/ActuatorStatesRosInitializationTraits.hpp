/*!
 * @file     ActuatorStatesRosInitializationTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/initialization_traits/InitializationTraits.hpp"

// series elastic actuator msgs
#include <series_elastic_actuator_msgs/SeActuatorStates.h>

namespace anymal_model_ros {
namespace initialization_traits {

template <>
class InitializationTraits<series_elastic_actuator_msgs::SeActuatorStates> {
 public:
  using RosType = series_elastic_actuator_msgs::SeActuatorStates;

  static void initialize(RosType& ros);
};

using ActuatorStatesRosInitializationTraits = InitializationTraits<series_elastic_actuator_msgs::SeActuatorStates>;

}  // namespace initialization_traits
}  // namespace anymal_model_ros
