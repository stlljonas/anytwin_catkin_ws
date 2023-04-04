/*
 * CustomCommandsManager.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "anymal_ctrl_free_gait/custom_commands/CustomCommandsManager.hpp"

#include <message_logger/message_logger.hpp>

namespace anymal_ctrl_free_gait {

using AD = anymal_description::AnymalDescription;

bool CustomCommandsManager::update() {
  if (!executor_.getQueue().hasStartedStep()) {
    return true;
  }
  const free_gait::Step& step = executor_.getQueue().getCurrentStep();
  if (!step.hasCustomCommand()) {
    reset();
    return true;
  }

  for (const free_gait::CustomCommand command : step.getCustomCommands()) {
    if (command.getType() == "empty") {
      bool success;
      emptyCommands_.emplace_back(command.getDuration(), success);
      if (!success) {
        MELO_ERROR_STREAM("Error when parsing Empty Custom Commands.");
        return false;
      }
    } else if (command.getType() == "freeze_joints") {
      bool success;
      freezeJointsCommands_.emplace_back(command.getCommand(), success);
      if (!success) {
        MELO_ERROR_STREAM("Error when parsing Freeze Joints Custom Commands.");
        return false;
      }
    } else {
      MELO_ERROR_STREAM("Custom command of type " << command.getType() << " not available.");
    }
  }
  return true;
}

bool CustomCommandsManager::updatePreController(const double dt) {
  for (auto iterator = std::begin(emptyCommands_); iterator != std::end(emptyCommands_); ++iterator) {
    if (!iterator->advance(dt)) {
      // TODO(pfankhauser): Should we delete the old elements also here?
    }
  }

  return true;
}

bool CustomCommandsManager::updatePostController(const double dt, loco_anymal::LegsAnymal& legs) {
  for (const auto& freezeJointsCommand : freezeJointsCommands_) {
    for (const auto& joint : freezeJointsCommand.getJointsToFreeze()) {
      const auto actuator = AD::mapEnums<AD::ActuatorEnum>(joint);
      const auto limb = AD::mapEnums<AD::LimbEnum>(actuator);
      const auto limbUInt = AD::mapKeyEnumToKeyId(AD::mapEnums<AD::LimbEnum>(limb));
      auto desiredLimbState = legs.getLegPtrById(limbUInt)->getLimbStateDesiredPtr();
      loco::JointControlModes jointControlModes = desiredLimbState->getJointControlModes();
      const auto jointNodeUInt = AD::mapKeyEnumToKeyId(AD::mapEnums<AD::JointNodeEnum>(actuator));
      jointControlModes(jointNodeUInt) = loco::ControlMode::MODE_FREEZE;
      desiredLimbState->setJointControlModes(jointControlModes);
    }
  }
  return true;
}

void CustomCommandsManager::reset() {
  emptyCommands_.clear();
  freezeJointsCommands_.clear();
}

} /* namespace anymal_ctrl_free_gait */
