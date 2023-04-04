/*
 * command_mode.hpp
 *
 *  Created on: Jul 18, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/typedefs.hpp>

// series elastic actuator
#include <series_elastic_actuator/SeActuatorCommand.hpp>

// message logger
#include <message_logger/message_logger.hpp>

// stl
#include <type_traits>
#include <map>

namespace command_mode {

using SeaMode = series_elastic_actuator::SeActuatorCommand::SeActuatorMode;
using LocoMode = loco::ControlMode;


template <LocoMode locoMode, SeaMode seaMode>
using locoModeToSeaModeKV = std_utils::KeyValuePair<LocoMode, SeaMode, locoMode, seaMode>;
using mapLocoModeToSeaMode =
std_utils::CompileTimeMap<LocoMode, SeaMode,
  locoModeToSeaModeKV<LocoMode::MODE_FREEZE, SeaMode::MODE_FREEZE>,
  locoModeToSeaModeKV<LocoMode::MODE_DISABLE, SeaMode::MODE_DISABLE>,
  locoModeToSeaModeKV<LocoMode::MODE_CURRENT, SeaMode::MODE_CURRENT>,
  locoModeToSeaModeKV<LocoMode::MODE_MOTOR_POSITION, SeaMode::MODE_MOTOR_POSITION>,
  locoModeToSeaModeKV<LocoMode::MODE_MOTOR_VELOCITY, SeaMode::MODE_MOTOR_VELOCITY>,
  locoModeToSeaModeKV<LocoMode::MODE_GEAR_POSITION, SeaMode::MODE_GEAR_POSITION>,
  locoModeToSeaModeKV<LocoMode::MODE_GEAR_VELOCITY, SeaMode::MODE_GEAR_VELOCITY>,
  locoModeToSeaModeKV<LocoMode::MODE_JOINT_POSITION, SeaMode::MODE_JOINT_POSITION>,
  locoModeToSeaModeKV<LocoMode::MODE_JOINT_VELOCITY, SeaMode::MODE_JOINT_VELOCITY>,
  locoModeToSeaModeKV<LocoMode::MODE_JOINT_TORQUE, SeaMode::MODE_JOINT_TORQUE>,
  locoModeToSeaModeKV<LocoMode::MODE_JOINT_POSITION_VELOCITY, SeaMode::MODE_JOINT_POSITION_VELOCITY>,
  locoModeToSeaModeKV<LocoMode::MODE_JOINT_POSITION_VELOCITY_TORQUE, SeaMode::MODE_JOINT_POSITION_VELOCITY_TORQUE>,
  locoModeToSeaModeKV<LocoMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS, SeaMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS>
>;

static constexpr SeaMode getSeaModeFromLocoMode(LocoMode mode) {
  return mapLocoModeToSeaMode::at(mode);
}

} /* namespace command_mode */
