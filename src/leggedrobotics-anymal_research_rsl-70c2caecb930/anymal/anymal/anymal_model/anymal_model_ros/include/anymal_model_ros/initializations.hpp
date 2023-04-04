#pragma once

// anymal model ros
#include "anymal_model_ros/initialization_traits/ActuatorCommandsRosInitializationTraits.hpp"
#include "anymal_model_ros/initialization_traits/ActuatorReadingsRosInitializationTraits.hpp"
#include "anymal_model_ros/initialization_traits/ActuatorStatesRosInitializationTraits.hpp"
#include "anymal_model_ros/initialization_traits/AnymalContactsRosInitializationTraits.hpp"
#include "anymal_model_ros/initialization_traits/AnymalFrameTransformsRosInitializationTraits.hpp"
#include "anymal_model_ros/initialization_traits/AnymalJointStatesRosInitializationTraits.hpp"
#include "anymal_model_ros/initialization_traits/AnymalStateRosInitializationTraits.hpp"
#include "anymal_model_ros/initialization_traits/ExtendedActuatorReadingsRosInitializationTraits.hpp"
#include "anymal_model_ros/initialization_traits/InitializationTraits.hpp"

namespace anymal_model_ros {

// Initialize a ROS message.
template <typename RosType>
inline void initialize(RosType& ros) {
  initialization_traits::InitializationTraits<RosType>::initialize(ros);
}

}  // namespace anymal_model_ros
