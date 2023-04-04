#pragma once

#include <string>

#include <xmlrpcpp/XmlRpc.h>

#include <anydrive/Command.hpp>
#include <anydrive/Reading.hpp>
#include <anydrive/ReadingExtended.hpp>
#include <anydrive/State.hpp>
#include <anydrive/StateExtended.hpp>
#include <anydrive/calibration/CalibrationModeEnum.hpp>
#include <anydrive/calibration/CalibrationTypeEnum.hpp>
#include <anydrive/configuration/Configuration.hpp>
#include <anydrive/fsm/StateEnum.hpp>
#include <anydrive/mode/ModeEnum.hpp>
#include <anydrive/setup/Setup.hpp>

#include <anydrive_msgs/CalibrationMode.h>
#include <anydrive_msgs/CalibrationType.h>
#include <anydrive_msgs/Command.h>
#include <anydrive_msgs/FsmState.h>
#include <anydrive_msgs/Mode.h>
#include <anydrive_msgs/Reading.h>
#include <anydrive_msgs/ReadingExtended.h>
#include <anydrive_msgs/State.h>
#include <anydrive_msgs/StateExtended.h>

namespace anydrive_ros {

anydrive::calibration::CalibrationTypeEnum calibrationTypeMsgToEnum(anydrive_msgs::CalibrationType calibrationTypeMsg);
anydrive_msgs::CalibrationType calibrationTypeEnumToMsg(anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum);

anydrive::calibration::CalibrationModeEnum calibrationModeMsgToEnum(anydrive_msgs::CalibrationMode calibrationModeMsg);
anydrive_msgs::CalibrationMode calibrationModeEnumToMsg(anydrive::calibration::CalibrationModeEnum calibrationModeEnum);

anydrive::mode::ModeEnum modeMsgToEnum(anydrive_msgs::Mode modeMsg);
anydrive_msgs::Mode modeEnumToMsg(anydrive::mode::ModeEnum modeEnum);

anydrive::fsm::StateEnum stateMsgToEnum(anydrive_msgs::FsmState fsmStateMsg);
anydrive_msgs::FsmState stateEnumToMsg(anydrive::fsm::StateEnum stateEnum);

void writeToMessage(anydrive_msgs::Command& message, const anydrive::Command& command);
void readFromMessage(anydrive::Command& command, const anydrive_msgs::Command& message);

void writeToMessage(anydrive_msgs::State& message, const anydrive::State& state);
void readFromMessage(anydrive::State& state, const anydrive_msgs::State& message);

void writeToMessage(anydrive_msgs::StateExtended& message, const anydrive::StateExtended& state);
void readFromMessage(anydrive::StateExtended& state, const anydrive_msgs::StateExtended& message);

void writeToMessage(anydrive_msgs::Reading& message, const anydrive::Reading& reading);
void readFromMessage(anydrive::Reading& reading, const anydrive_msgs::Reading& message);

void writeToMessage(anydrive_msgs::ReadingExtended& message, const anydrive::ReadingExtended& reading);
void readFromMessage(anydrive::ReadingExtended& reading, const anydrive_msgs::ReadingExtended& message);

bool readConfigurationParameters(XmlRpc::XmlRpcValue& params, anydrive::configuration::Configuration& configuration);
bool readAnydriveSetupParameters(XmlRpc::XmlRpcValue& params, anydrive::setup::AnydrivePtr& anydrivePtr);

}  // namespace anydrive_ros
