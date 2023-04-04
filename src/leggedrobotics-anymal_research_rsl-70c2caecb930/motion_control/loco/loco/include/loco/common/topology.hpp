/*
 * topology.hpp
 *
 *  Created on: Mar 21, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// stl
#include <map>
#include <string>

// loco
#include <loco/common/typedefs.hpp>

// std utils
#include <std_utils/std_utils.hpp>

namespace loco {

CONSECUTIVE_ENUM(ControlMode, MODE_FREEZE, MODE_DISABLE, MODE_CURRENT, MODE_MOTOR_POSITION, MODE_MOTOR_VELOCITY, MODE_GEAR_POSITION,
                 MODE_GEAR_VELOCITY, MODE_JOINT_POSITION, MODE_JOINT_VELOCITY, MODE_JOINT_TORQUE, MODE_JOINT_POSITION_VELOCITY,
                 MODE_JOINT_POSITION_VELOCITY_TORQUE, MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS, MODE_UNDEFINED)

namespace internal {

static constexpr std_utils::KeyArray<ControlMode> controlModeKeys{
    {std_utils::make_key(ControlMode::MODE_FREEZE, "freeze"), std_utils::make_key(ControlMode::MODE_DISABLE, "disable"),
     std_utils::make_key(ControlMode::MODE_CURRENT, "current"), std_utils::make_key(ControlMode::MODE_MOTOR_POSITION, "motor_position"),
     std_utils::make_key(ControlMode::MODE_MOTOR_VELOCITY, "motor_velocity"),
     std_utils::make_key(ControlMode::MODE_GEAR_POSITION, "gear_position"),
     std_utils::make_key(ControlMode::MODE_GEAR_VELOCITY, "gear_velocity"),
     std_utils::make_key(ControlMode::MODE_JOINT_POSITION, "joint_position"),
     std_utils::make_key(ControlMode::MODE_JOINT_VELOCITY, "joint_velocity"),
     std_utils::make_key(ControlMode::MODE_JOINT_TORQUE, "joint_torque"),
     std_utils::make_key(ControlMode::MODE_JOINT_POSITION_VELOCITY, "joint_position_velocity"),
     std_utils::make_key(ControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE, "joint_position_velocity_torque"),
     std_utils::make_key(ControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS, "joint_position_velocity_torque_pid_gains"),
     std_utils::make_key(ControlMode::MODE_UNDEFINED, "undefined")}};

}  // namespace internal

inline static constexpr const std_utils::KeyArray<ControlMode> getControlModeKeys() {
  return loco::internal::controlModeKeys;
}

}  // namespace loco
