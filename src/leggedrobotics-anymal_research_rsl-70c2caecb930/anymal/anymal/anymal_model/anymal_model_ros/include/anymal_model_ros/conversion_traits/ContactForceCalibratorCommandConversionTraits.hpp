/*!
 * @file     ActuatorCommandsRosConversionTraits.hpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/conversion_traits/ConversionTraits.hpp"
#include "anymal_model_ros/typedefs.hpp"

// robot utils ros
#include <robot_utils_ros/force_calibrators/ConvertRosMessages.hpp>

// anymal model
#include <anymal_model/contact_force_calibrator_containers.hpp>

// robot utils ros
#include <robot_utils_ros/ForceCalibratorCommands.h>

namespace anymal_model_ros {
namespace conversion_traits {

template <>
class ConversionTraits<anymal_model::ContactForceCalibratorCommandContainer, robot_utils_ros::ForceCalibratorCommands>
    : public robot_utils_ros::ConversionTraits<AD, anymal_model::ContactForceCalibratorCommandContainer,
                                               robot_utils_ros::ForceCalibratorCommands> {};

}  // namespace conversion_traits
}  // namespace anymal_model_ros
