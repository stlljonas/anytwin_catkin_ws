/**
 * @file    contact_force_calibrator_containers.hpp
 * @author  Dario Bellicoso
 * @date    Nov, 2017
 * @version 1.0
 */

#pragma once

// anymal model
#include "anymal_model/typedefs.hpp"

// romo std
#include <romo_std/common/actuator_containers.hpp>

// robot utils
#include <robot_utils/force_calibrators/ForceCalibratorCommand.hpp>
#include <robot_utils/force_calibrators/ForceCalibratorStats.hpp>

namespace anymal_model {

using ContactForceCalibratorCommandContainer =
    std_utils::EnumArray<CAD::ConcreteTopology::ContactEnum, robot_utils::ForceCalibratorCommand>;
using ContactForceCalibratorStatsContainer = std_utils::EnumArray<CAD::ConcreteTopology::ContactEnum, robot_utils::ForceCalibratorStats>;

}  // namespace anymal_model
