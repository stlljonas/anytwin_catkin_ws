#pragma once


// anydrive
#include <anydrive/Command.hpp>
#include <anydrive/Reading.hpp>
#include <anydrive/ReadingExtended.hpp>
#include <anydrive/State.hpp>
#include <anydrive/StateExtended.hpp>

// series elastic actuator
#include <series_elastic_actuator/SeActuatorCommand.hpp>
#include <series_elastic_actuator/SeActuatorReading.hpp>
#include <series_elastic_actuator/SeActuatorReadingExtended.hpp>
#include <series_elastic_actuator/SeActuatorState.hpp>
#include <series_elastic_actuator/SeActuatorStateExtended.hpp>


namespace series_elastic_actuator_anydrive {


using AnydriveMode = anydrive::mode::ModeEnum;
using AnydriveCommand = anydrive::Command;
using AnydriveState = anydrive::State;
using AnydriveStateExtended = anydrive::StateExtended;
using AnydriveReading = anydrive::Reading;
using AnydriveReadingExtended = anydrive::ReadingExtended;

using SeActuatorMode = series_elastic_actuator::SeActuatorCommand::SeActuatorMode;
using SeActuatorCommand = series_elastic_actuator::SeActuatorCommand;
using SeActuatorState = series_elastic_actuator::SeActuatorState;
using SeActuatorStateExtended = series_elastic_actuator::SeActuatorStateExtended;
using SeActuatorReading = series_elastic_actuator::SeActuatorReading;
using SeActuatorReadingExtended = series_elastic_actuator::SeActuatorReadingExtended;


void anydriveToSeActuator(SeActuatorMode& seActuatorMode, const AnydriveMode anydriveMode);
void seActuatorToAnydrive(AnydriveMode& anydriveMode, const SeActuatorMode seActuatorMode);

void anydriveToSeActuator(SeActuatorCommand& seActuatorCommand, const AnydriveCommand& anydriveCommand);
void seActuatorToAnydrive(AnydriveCommand& anydriveCommand, const SeActuatorCommand& seActuatorCommand);

void anydriveToSeActuator(SeActuatorState& seActuatorState, const AnydriveState& anydriveState);
void seActuatorToAnydrive(AnydriveState& anydriveState, const SeActuatorState& seActuatorState);

void anydriveToSeActuator(SeActuatorStateExtended& seActuatorStateExtended, const AnydriveStateExtended& anydriveStateExtended);
void seActuatorToAnydrive(AnydriveStateExtended& anydriveStateExtended, const SeActuatorStateExtended& seActuatorStateExtended);

void anydriveToSeActuator(SeActuatorReading& seActuatorReading, const AnydriveReading& anydriveReading);
void seActuatorToAnydrive(AnydriveReading& anydriveReading, const SeActuatorReading& seActuatorReading);

void anydriveToSeActuator(SeActuatorReadingExtended& seActuatorReadingExtended, const AnydriveReadingExtended& anydriveReadingExtended);
void seActuatorToAnydrive(AnydriveReadingExtended& anydriveReadingExtended, const SeActuatorReadingExtended& seActuatorReadingExtended);


} // series_elastic_actuator_anydrive
