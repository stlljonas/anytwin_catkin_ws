/*!
 * @file    series_elastic_actuator_readings.hpp
 * @author  Markus Staeuble
 * @date    Mar 7, 2018
 *
 */

#pragma once

#include <series_elastic_actuator_msgs/SeActuatorReadings.h>
#include <series_elastic_actuator_msgs/SeActuatorReadingsExtended.h>

namespace series_elastic_actuator_ros {

template<typename ConcreteDescription_>
static void initializeSeActuatorReadings(series_elastic_actuator_msgs::SeActuatorReadingsExtended& readings) {
  using SeActuatorEnum = typename ConcreteDescription_::ConcreteTopology::SeActuatorEnum;
  constexpr auto numSeActuators = static_cast<unsigned int>(SeActuatorEnum::SIZE);

  readings.readings.clear();
  readings.readings.resize(numSeActuators);
  for (const auto& key : ConcreteDescription_::template getKeys<SeActuatorEnum>()) {
    const auto actuatorId = key.getId();
    readings.readings[actuatorId].state.header.frame_id = key.getName();
    readings.readings[actuatorId].state.name = key.getName();
    readings.readings[actuatorId].commanded.mode = series_elastic_actuator_msgs::SeActuatorCommand::MODE_FREEZE;
    readings.readings[actuatorId].commanded.name = key.getName();
  }
}

template<typename ConcreteDescription_>
static void initializeSeActuatorReadings(series_elastic_actuator_msgs::SeActuatorReadings& readings) {
  using SeActuatorEnum = typename ConcreteDescription_::ConcreteTopology::SeActuatorEnum;
  constexpr auto numSeActuators = static_cast<unsigned int>(SeActuatorEnum::SIZE);

  readings.readings.clear();
  readings.readings.resize(numSeActuators);
  for (const auto& key : ConcreteDescription_::template getKeys<SeActuatorEnum>()) {
    const auto actuatorId = key.getId();
    readings.readings[actuatorId].state.header.frame_id = key.getName();
    readings.readings[actuatorId].state.name = key.getName();
    readings.readings[actuatorId].commanded.mode = series_elastic_actuator_msgs::SeActuatorCommand::MODE_FREEZE;
    readings.readings[actuatorId].commanded.name = key.getName();
  }
}

}