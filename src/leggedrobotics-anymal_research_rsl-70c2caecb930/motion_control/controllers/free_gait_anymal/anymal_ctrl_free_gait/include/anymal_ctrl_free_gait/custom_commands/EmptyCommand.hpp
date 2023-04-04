/*
 * EmptyCommand.hpp
 *
 *  Created on: Sep 2, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

namespace anymal_ctrl_free_gait {

class EmptyCommand {
 public:
  EmptyCommand(const double duration, bool& success);
  virtual ~EmptyCommand() = default;

  double getDuration() const;

  bool advance(const double dt);

 private:
  double duration_;
};

}  // namespace anymal_ctrl_free_gait
