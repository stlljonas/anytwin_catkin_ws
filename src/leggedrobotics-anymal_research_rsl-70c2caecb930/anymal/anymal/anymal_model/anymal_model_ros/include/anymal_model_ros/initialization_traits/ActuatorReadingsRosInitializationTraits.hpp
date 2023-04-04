/*!
 * @file     ActuatorReadingsRosInitializationTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/initialization_traits/InitializationTraits.hpp"

// series elastic actuator msgs
#include <series_elastic_actuator_msgs/SeActuatorReadings.h>

namespace anymal_model_ros {
namespace initialization_traits {

template <>
class InitializationTraits<series_elastic_actuator_msgs::SeActuatorReadings> {
 public:
  using RosType = series_elastic_actuator_msgs::SeActuatorReadings;

  static void initialize(RosType& ros);
};

using ActuatorReadingsRosInitializationTraits = InitializationTraits<series_elastic_actuator_msgs::SeActuatorReadings>;

}  // namespace initialization_traits
}  // namespace anymal_model_ros
