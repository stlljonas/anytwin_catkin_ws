/*
 * StateSwitcherBase.hpp
 *
 *  Created on: Oct 5, 2014
 *      Author: Dario Bellicoso
 */

#pragma once

namespace loco {

class StateSwitcherBase {
 public:
  StateSwitcherBase() = default;
  virtual ~StateSwitcherBase() = default;

  virtual bool initialize(double dt) = 0;
};

} /* namespace loco */