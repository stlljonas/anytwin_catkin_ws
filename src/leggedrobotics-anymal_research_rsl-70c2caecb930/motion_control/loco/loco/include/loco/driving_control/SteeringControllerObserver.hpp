/*
 * SteeringControllerObserver.hpp
 *
 *  Created on: Apr 6, 2017
 *      Author: gabrielhottiger
 */

#pragma once

#include <string>

namespace loco {

class SteeringControllerObserver {
 public:
  virtual ~SteeringControllerObserver() = default;
  virtual void steeringModeChanged(const std::string& steeringMode) = 0;
};

}  // namespace loco
