/*
 * FreezeJointsCommand.hpp
 *
 *  Created on: Feb 15, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include <anymal_description/AnymalDescription.hpp>

#include <string>
#include <vector>

namespace anymal_ctrl_free_gait {

class FreezeJointsCommand {
 private:
  using AD = anymal_description::AnymalDescription;

 public:
  FreezeJointsCommand(const std::string& command, bool& success);
  virtual ~FreezeJointsCommand() = default;

  const std::vector<AD::JointEnum>& getJointsToFreeze() const;

 private:
  std::vector<AD::JointEnum> jointsToFreeze_;
};

}  // namespace anymal_ctrl_free_gait
