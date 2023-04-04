/**
 * @file    actuator_containers.hpp
 * @author  Dario Bellicoso
 * @date    Nov, 2017
 * @version 1.0
 */

#pragma once

// anymal model
#include "anymal_model/LegConfigurations.hpp"
#include "anymal_model/LimitsAnymal.hpp"
#include "anymal_model/typedefs.hpp"

// series elastic actuator
#include <series_elastic_actuator/SeActuatorCommand.hpp>
#include <series_elastic_actuator/SeActuatorReading.hpp>

// romo std
#include <romo_std/common/actuator_containers.hpp>

namespace anymal_model {

using ActuatorCommandRobotContainer = romo_std::ActuatorCommandContainer<CAD, series_elastic_actuator::SeActuatorCommand>;
using ActuatorCommandPtrBranchNodeContainer =
    romo_std::ActuatorCommandPtrBranchNodeContainer<CAD, series_elastic_actuator::SeActuatorCommand>;
using ActuatorCommandPtrNodeBranchContainer =
    romo_std::ActuatorCommandPtrNodeBranchContainer<CAD, series_elastic_actuator::SeActuatorCommand>;

using ActuatorReadingRobotContainer = romo_std::ActuatorReadingContainer<CAD, series_elastic_actuator::SeActuatorReading>;
using ActuatorReadingPtrBranchNodeContainer =
    romo_std::ActuatorReadingPtrBranchNodeContainer<CAD, series_elastic_actuator::SeActuatorReading>;
using ActuatorReadingPtrNodeBranchContainer =
    romo_std::ActuatorReadingPtrNodeBranchContainer<CAD, series_elastic_actuator::SeActuatorReading>;

void initializeActuatorCommandsFromLimits(ActuatorCommandRobotContainer& actuatorCommands, const LimitsAnymal& limits,
                                          const LegConfigurations& legConfigAnymal);

}  // namespace anymal_model
