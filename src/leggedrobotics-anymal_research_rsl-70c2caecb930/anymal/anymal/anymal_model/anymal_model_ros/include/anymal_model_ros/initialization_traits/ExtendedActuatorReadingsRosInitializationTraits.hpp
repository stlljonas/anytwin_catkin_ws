/*!
 * @file     ExtendedActuatorReadingsRosInitializationTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/initialization_traits/InitializationTraits.hpp"

// series elastic actuator msgs
#include <series_elastic_actuator_msgs/SeActuatorReadingsExtended.h>

namespace anymal_model_ros {
namespace initialization_traits {

template <>
class InitializationTraits<series_elastic_actuator_msgs::SeActuatorReadingsExtended> {
 public:
  using RosType = series_elastic_actuator_msgs::SeActuatorReadingsExtended;

  static void initialize(RosType& ros);
};

using ExtendedActuatorReadingsRosInitializationTraits = InitializationTraits<series_elastic_actuator_msgs::SeActuatorReadingsExtended>;

}  // namespace initialization_traits
}  // namespace anymal_model_ros
