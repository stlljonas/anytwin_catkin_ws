/*!
 * @file     ActuatorReadingsRosInitializationTraits.cpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

// anymal model ros
#include "anymal_model_ros/initialization_traits/ActuatorReadingsRosInitializationTraits.hpp"
#include "anymal_model_ros/typedefs.hpp"

namespace anymal_model_ros {
namespace initialization_traits {

void ActuatorReadingsRosInitializationTraits::initialize(RosType& ros) {
  constexpr auto numActuators = AD::getActuatorsDimension();
  ros.readings.resize(numActuators);
  for (const auto& key : AD::getActuatorKeys()) {
    const auto actuatorId = key.getId();
    ros.readings[actuatorId].state.header.frame_id = key.getName();
    ros.readings[actuatorId].state.name = key.getName();

    ros.readings[actuatorId].commanded.mode = series_elastic_actuator_msgs::SeActuatorCommand::MODE_FREEZE;
    ros.readings[actuatorId].commanded.position = 0.0;
    ros.readings[actuatorId].commanded.velocity = 0.0;
    ros.readings[actuatorId].commanded.joint_torque = 0.0;
    ros.readings[actuatorId].commanded.name = key.getName();
  }
}

}  // namespace initialization_traits
}  // namespace anymal_model_ros
