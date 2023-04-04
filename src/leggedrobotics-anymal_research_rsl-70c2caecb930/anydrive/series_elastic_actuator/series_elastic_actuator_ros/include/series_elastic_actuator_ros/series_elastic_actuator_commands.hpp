/*!
 * @file    series_elastic_actuator_commands.hpp
 * @author  Markus Staeuble
 * @date    Mar 7, 2018
 *
 */

#pragma once

#include <series_elastic_actuator_msgs/SeActuatorCommands.h>

namespace series_elastic_actuator_ros {

template<typename ConcreteDescription_>
static void initializeSeActuatorCommands(series_elastic_actuator_msgs::SeActuatorCommands& commands)  {
  using SeActuatorEnum = typename ConcreteDescription_::ConcreteTopology::SeActuatorEnum;
  constexpr auto numSeActuators = static_cast<unsigned int>(SeActuatorEnum::SIZE);

  commands.commands.clear();
  commands.commands.resize(numSeActuators);
  for (const auto & key : ConcreteDescription_::template getKeys<SeActuatorEnum>()) {
    const auto commandId = key.getId();
    commands.commands[commandId].mode = series_elastic_actuator_msgs::SeActuatorCommand::MODE_FREEZE;
    commands.commands[commandId].name = key.getName();
    commands.commands[commandId].header.frame_id  = key.getName();
  }
}

}