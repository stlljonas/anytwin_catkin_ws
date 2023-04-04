#pragma once


// series elastic actuator
#include <series_elastic_actuator/common.hpp>
#include <series_elastic_actuator/SeActuatorCommand.hpp>
#include <series_elastic_actuator/SeActuatorReading.hpp>
#include <series_elastic_actuator/SeActuatorReadingExtended.hpp>
#include <series_elastic_actuator/SeActuatorState.hpp>
#include <series_elastic_actuator/SeActuatorStateExtended.hpp>


namespace series_elastic_actuator_sim {


using series_elastic_actuator::saturate;

using SeActuatorCommand = series_elastic_actuator::SeActuatorCommand;
using SeActuatorReading = series_elastic_actuator::SeActuatorReading;
using SeActuatorReadingExtended = series_elastic_actuator::SeActuatorReadingExtended;
using SeActuatorState = series_elastic_actuator::SeActuatorState;
using SeActuatorStateExtended = series_elastic_actuator::SeActuatorStateExtended;


} // series_elastic_actuator_sim
