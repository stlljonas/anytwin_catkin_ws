/*!
 * @file     ActuatorCommandsRosInitializationTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/initialization_traits/InitializationTraits.hpp"

// series elastic actuator msgs
#include <series_elastic_actuator_msgs/SeActuatorCommands.h>

namespace anymal_model_ros {
namespace initialization_traits {

template <>
class InitializationTraits<series_elastic_actuator_msgs::SeActuatorCommands> {
 public:
  using RosType = series_elastic_actuator_msgs::SeActuatorCommands;

  static void initialize(RosType& ros);
};

using ActuatorCommandsRosInitializationTraits = InitializationTraits<series_elastic_actuator_msgs::SeActuatorCommands>;

}  // namespace initialization_traits
}  // namespace anymal_model_ros
