/*
 * EmptyCommand.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <anymal_ctrl_free_gait/custom_commands/EmptyCommand.hpp>
#include <iostream>

namespace anymal_ctrl_free_gait {

EmptyCommand::EmptyCommand(const double duration, bool& success) : duration_(0.0) {
  duration_ = duration;
  success = true;
}

double EmptyCommand::getDuration() const {
  return duration_;
}

bool EmptyCommand::advance(const double dt) {
  duration_ -= dt;
  return (duration_ > 0.0);
}

}  // namespace anymal_ctrl_free_gait
