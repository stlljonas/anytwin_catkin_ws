#pragma once

// anymal model ros
#include "anymal_model_ros/conversion_traits/ActuatorCommandsRosConversionTraits.hpp"
#include "anymal_model_ros/conversion_traits/ActuatorReadingsRosConversionTraits.hpp"
#include "anymal_model_ros/conversion_traits/AnymalJointStatesRosConversionTraits.hpp"
#include "anymal_model_ros/conversion_traits/AnymalModelRosConversionTraits.hpp"
#include "anymal_model_ros/conversion_traits/AnymalStateRosConversionTraits.hpp"
#include "anymal_model_ros/conversion_traits/ContactForceCalibratorCommandConversionTraits.hpp"
#include "anymal_model_ros/conversion_traits/ConversionTraits.hpp"
#include "anymal_model_ros/conversion_traits/ExtendedAnymalStateRosConversionTraits.hpp"

namespace anymal_model_ros {

// Conversion from object to ROS message filling an existing ROS message.
template <typename ObjectType, typename RosType>
inline void toRos(const ObjectType& object, RosType& ros) {
  conversion_traits::ConversionTraits<ObjectType, RosType>::convert(object, ros);
}

// Conversion from object to ROS message creating a new ROS message.
template <typename ObjectType, typename RosType>
inline RosType toRos(const ObjectType& object) {
  return conversion_traits::ConversionTraits<ObjectType, RosType>::convert(object);
}

// Conversion from ROS message to object filling an existing object.
template <typename ObjectType, typename RosType>
inline void fromRos(const RosType& ros, ObjectType& object) {
  conversion_traits::ConversionTraits<ObjectType, RosType>::convert(ros, object);
}

// Conversion from ROS message to object creating a new object.
template <typename ObjectType, typename RosType>
inline ObjectType fromRos(const RosType& ros) {
  return conversion_traits::ConversionTraits<ObjectType, RosType>::convert(ros);
}

}  // namespace anymal_model_ros
