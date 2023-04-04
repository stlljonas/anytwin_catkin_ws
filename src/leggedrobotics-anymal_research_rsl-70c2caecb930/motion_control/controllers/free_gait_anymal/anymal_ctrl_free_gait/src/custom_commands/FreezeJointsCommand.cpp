/*
 * FreezeJointsCommand.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "anymal_ctrl_free_gait/custom_commands/FreezeJointsCommand.hpp"

#include <yaml_tools/YamlNode.hpp>

#include <iostream>

namespace anymal_ctrl_free_gait {

FreezeJointsCommand::FreezeJointsCommand(const std::string& command, bool& success) {
  const yaml_tools::YamlNode data = yaml_tools::YamlNode::fromString(command);
  if (!data.isSequence()) {
    success = false;
    return;
  }

  jointsToFreeze_.clear();
  jointsToFreeze_.resize(data.size());
  for (size_t i = 0; i < data.size(); ++i) {  // NOLINT(modernize-loop-convert)
    jointsToFreeze_.push_back(AD::mapKeyNameToKeyEnum<AD::JointEnum>(data[i].as<std::string>()));
  }

  success = true;
}

const std::vector<FreezeJointsCommand::AD::JointEnum>& FreezeJointsCommand::getJointsToFreeze() const {
  return jointsToFreeze_;
}

}  // namespace anymal_ctrl_free_gait
