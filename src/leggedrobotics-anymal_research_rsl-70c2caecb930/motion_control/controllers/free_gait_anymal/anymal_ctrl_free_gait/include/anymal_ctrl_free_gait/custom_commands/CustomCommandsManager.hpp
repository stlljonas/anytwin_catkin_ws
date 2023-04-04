/*
 * CustomCommandsManager.hpp
 *
 *  Created on: Feb 15, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "anymal_ctrl_free_gait/custom_commands/EmptyCommand.hpp"
#include "anymal_ctrl_free_gait/custom_commands/FreezeJointsCommand.hpp"

#include <free_gait_core/executor/Executor.hpp>
#include <loco_anymal/common/LegsAnymal.hpp>

#include <memory>

namespace anymal_ctrl_free_gait {

class CustomCommandsManager {
 public:
  explicit CustomCommandsManager(free_gait::Executor& executor) : executor_(executor) {}
  virtual ~CustomCommandsManager() = default;

  bool update();
  bool updatePreController(const double dt);
  bool updatePostController(const double dt, loco_anymal::LegsAnymal& legs);
  void reset();

 private:
  free_gait::Executor& executor_;
  std::vector<EmptyCommand> emptyCommands_;
  std::vector<FreezeJointsCommand> freezeJointsCommands_;
};

}  // namespace anymal_ctrl_free_gait
