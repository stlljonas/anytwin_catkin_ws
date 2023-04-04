/*!
 * @file    ConvertRosMessages.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
#pragma once


// series elastic actuator
#include <series_elastic_actuator/SeActuatorCommand.hpp>
#include <series_elastic_actuator/SeActuatorReading.hpp>
#include <series_elastic_actuator/SeActuatorReadingExtended.hpp>
#include <series_elastic_actuator/SeActuatorState.hpp>
#include <series_elastic_actuator/SeActuatorStateExtended.hpp>

// series elastic actuator msgs
#include <series_elastic_actuator_msgs/SeActuatorCommand.h>
#include <series_elastic_actuator_msgs/SeActuatorCommands.h>
#include <series_elastic_actuator_msgs/SeActuatorReading.h>
#include <series_elastic_actuator_msgs/SeActuatorReadings.h>
#include <series_elastic_actuator_msgs/SeActuatorReadingExtended.h>
#include <series_elastic_actuator_msgs/SeActuatorState.h>
#include <series_elastic_actuator_msgs/SeActuatorStateExtended.h>

#include <array>
#include <std_utils/std_utils.hpp>

namespace series_elastic_actuator_ros {


class ConvertRosMessages
{
protected:
  ConvertRosMessages() {}
  virtual ~ConvertRosMessages() {}

public:
  static void writeToMessage(series_elastic_actuator_msgs::SeActuatorCommand& message, const series_elastic_actuator::SeActuatorCommand& command);
  static void readFromMessage(series_elastic_actuator::SeActuatorCommand& command, const series_elastic_actuator_msgs::SeActuatorCommand& message);

  static void writeToMessage(series_elastic_actuator_msgs::SeActuatorState& message, const series_elastic_actuator::SeActuatorState& state);
  static void readFromMessage(series_elastic_actuator::SeActuatorState& state, const series_elastic_actuator_msgs::SeActuatorState& message);

  static void writeToMessage(series_elastic_actuator_msgs::SeActuatorReading& message, const series_elastic_actuator::SeActuatorReading& reading);
  static void readFromMessage(series_elastic_actuator::SeActuatorReading& reading, const series_elastic_actuator_msgs::SeActuatorReading& message);

  static void writeToMessage(series_elastic_actuator_msgs::SeActuatorStateExtended& message, const series_elastic_actuator::SeActuatorStateExtended& state);
  static void readFromMessage(series_elastic_actuator::SeActuatorStateExtended& state, const series_elastic_actuator_msgs::SeActuatorStateExtended& message);

  static void writeToMessage(series_elastic_actuator_msgs::SeActuatorReadingExtended& message, const series_elastic_actuator::SeActuatorReadingExtended& reading);
  static void readFromMessage(series_elastic_actuator::SeActuatorReadingExtended& reading, const series_elastic_actuator_msgs::SeActuatorReadingExtended& message);
};

template<typename ConcreteDescription_, typename Msg_, typename MsgRos_>
class ConversionTraits;

//! This conversion is used by cosmo
template<typename ConcreteDescription_>
class ConversionTraits<ConcreteDescription_, std_utils::EnumArray<typename ConcreteDescription_::ConcreteTopology::SeActuatorEnum, series_elastic_actuator::SeActuatorReading>, series_elastic_actuator_msgs::SeActuatorReadings> {
 public:
  using SeActuatorEnum = typename ConcreteDescription_::ConcreteTopology::SeActuatorEnum;
  using Msg = std_utils::EnumArray<SeActuatorEnum, series_elastic_actuator::SeActuatorReading>;
  using MsgRos = series_elastic_actuator_msgs::SeActuatorReadings;

  inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    series_elastic_actuator_msgs::SeActuatorReading reading;
    rosMsg.readings.reserve(ConcreteDescription_::template getKeys<SeActuatorEnum>().size());
    for (auto actuatorKey : ConcreteDescription_::template getKeys<SeActuatorEnum>()) {
      ConvertRosMessages::writeToMessage(reading, msg[actuatorKey.getEnum()]);
      rosMsg.readings.push_back(reading);
    }
    return rosMsg;
  }

  inline static Msg convert(const MsgRos& rosMsg) {
    Msg msg;
    assert(rosMsg.readings.size() == ConcreteDescription_::getActuatorsDimension());
    for (auto actuatorKey : ConcreteDescription_::template getKeys<SeActuatorEnum>()) {
      ConvertRosMessages::readFromMessage(msg[actuatorKey.getEnum()], rosMsg.readings[actuatorKey.getId()]);
    }
    return msg;
  }
};

//! This conversion is used by cosmo
template<typename ConcreteDescription_>
class ConversionTraits<ConcreteDescription_, std_utils::EnumArray<typename ConcreteDescription_::ConcreteTopology::SeActuatorEnum, series_elastic_actuator::SeActuatorCommand>, series_elastic_actuator_msgs::SeActuatorCommands> {
 public:
  using SeActuatorEnum = typename ConcreteDescription_::ConcreteTopology::SeActuatorEnum;
  using Msg = std_utils::EnumArray<SeActuatorEnum, series_elastic_actuator::SeActuatorCommand>;
  using MsgRos = series_elastic_actuator_msgs::SeActuatorCommands;

  inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    series_elastic_actuator_msgs::SeActuatorCommand command;
    rosMsg.commands.reserve(ConcreteDescription_::template getKeys<SeActuatorEnum>().size());
    for (auto actuatorKey : ConcreteDescription_::template getKeys<SeActuatorEnum>()) {
      ConvertRosMessages::writeToMessage(command, msg[actuatorKey.getEnum()]);
      rosMsg.commands.push_back(command);
    }
    return rosMsg;
  }

  inline static Msg convert(const MsgRos& rosMsg) {
    Msg msg;
    assert(rosMsg.commands.size() == ConcreteDescription_::getActuatorsDimension() );
    for (auto actuatorKey : ConcreteDescription_::template getKeys<SeActuatorEnum>()) {
      ConvertRosMessages::readFromMessage(msg[actuatorKey.getEnum()], rosMsg.commands[actuatorKey.getId()]);
    }
    return msg;
  }
};


//! This conversion is used by cosmo
template<typename ConcreteDescription_>
class ConversionTraits<ConcreteDescription_, std_utils::EnumArray<typename ConcreteDescription_::ActuatorEnum, series_elastic_actuator::SeActuatorReading>, series_elastic_actuator_msgs::SeActuatorReadings> {
 public:
  typedef std_utils::EnumArray<typename ConcreteDescription_::ActuatorEnum, series_elastic_actuator::SeActuatorReading> Msg;
  typedef series_elastic_actuator_msgs::SeActuatorReadings MsgRos;

  inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    series_elastic_actuator_msgs::SeActuatorReading reading;
    for (auto actuatorKey : ConcreteDescription_::getActuatorKeys()) {
      ConvertRosMessages::writeToMessage(reading, msg[actuatorKey.getEnum()]);
      rosMsg.readings.push_back(reading);
    }
    return rosMsg;
  }

  inline static Msg convert(const MsgRos& rosMsg) {
    Msg msg;
    assert(rosMsg.readings.size() == ConcreteDescription_::getActuatorsDimension());
    for (auto actuatorKey : ConcreteDescription_::getActuatorKeys()) {
      ConvertRosMessages::readFromMessage(msg[actuatorKey.getEnum()], rosMsg.readings[actuatorKey.getId()]);
    }
    return msg;
  }
};


//! This conversion is used by cosmo
template<typename ConcreteDescription_>
class ConversionTraits<ConcreteDescription_, std_utils::EnumArray<typename ConcreteDescription_::ActuatorEnum, series_elastic_actuator::SeActuatorCommand>, series_elastic_actuator_msgs::SeActuatorCommands> {
 public:
  typedef std_utils::EnumArray<typename ConcreteDescription_::ActuatorEnum, series_elastic_actuator::SeActuatorCommand> Msg;
  typedef series_elastic_actuator_msgs::SeActuatorCommands MsgRos;

  inline static MsgRos convert(const Msg& msg) {
    MsgRos rosMsg;
    series_elastic_actuator_msgs::SeActuatorCommand command;
    for (auto actuatorKey : ConcreteDescription_::getActuatorKeys()) {
      ConvertRosMessages::writeToMessage(command, msg[actuatorKey.getEnum()]);
      rosMsg.commands.push_back(command);
    }
    return rosMsg;
  }

  inline static Msg convert(const MsgRos& rosMsg) {
    Msg msg;
    assert(rosMsg.commands.size() == ConcreteDescription_::getActuatorsDimension() );
    for (auto actuatorKey : ConcreteDescription_::getActuatorKeys()) {
      ConvertRosMessages::readFromMessage(msg[actuatorKey.getEnum()], rosMsg.commands[actuatorKey.getId()]);
    }
    return msg;
  }
};

} // series_elastic_actuator_ros
