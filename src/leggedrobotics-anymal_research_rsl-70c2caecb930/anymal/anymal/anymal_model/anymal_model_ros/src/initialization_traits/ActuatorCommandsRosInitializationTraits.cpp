/*!
 * @file     ActuatorCommandsRosInitializationTraits.cpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

// anymal model ros
#include "anymal_model_ros/initialization_traits/ActuatorCommandsRosInitializationTraits.hpp"
#include "anymal_model_ros/typedefs.hpp"

namespace anymal_model_ros {
namespace initialization_traits {

void ActuatorCommandsRosInitializationTraits::initialize(RosType& ros) {
  constexpr auto numActuators = AD::getActuatorsDimension();
  ros.commands.resize(numActuators);
  for (const auto& key : AD::getActuatorKeys()) {
    const auto actuatorId = key.getId();
    ros.commands[actuatorId].mode = series_elastic_actuator_msgs::SeActuatorCommand::MODE_FREEZE;
    ros.commands[actuatorId].position = 0.0;
    ros.commands[actuatorId].velocity = 0.0;
    ros.commands[actuatorId].joint_torque = 0.0;
    ros.commands[actuatorId].name = key.getName();
    ros.commands[actuatorId].header.frame_id = key.getName();
  }
}

}  // namespace initialization_traits
}  // namespace anymal_model_ros
