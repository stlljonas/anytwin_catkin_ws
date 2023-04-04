/*!
 * @file     ActuatorReadingsRosConversionTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/conversion_traits/ConversionTraits.hpp"
#include "anymal_model_ros/typedefs.hpp"

// series elastic actuator ros
#include <series_elastic_actuator_ros/ConvertRosMessages.hpp>

// anymal model
#include <anymal_model/actuator_containers.hpp>

// series elastic actuator msgs
#include <series_elastic_actuator_msgs/SeActuatorReadings.h>

namespace anymal_model_ros {
namespace conversion_traits {

template <>
class ConversionTraits<anymal_model::ActuatorReadingRobotContainer, series_elastic_actuator_msgs::SeActuatorReadings>
    : public series_elastic_actuator_ros::ConversionTraits<AD, anymal_model::ActuatorReadingRobotContainer,
                                                           series_elastic_actuator_msgs::SeActuatorReadings> {};

}  // namespace conversion_traits
}  // namespace anymal_model_ros
