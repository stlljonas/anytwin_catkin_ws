/*
 * HandStateDesired.hpp
 *
 *  Created on: June 25, 2018
 *      Author: Markus Staeuble
 */

#pragma once

// loco
#include <loco/common/end_effectors/EndEffectorStateDesired.hpp>
#include <loco/common/end_effectors/HandStateBase.hpp>
#include <loco/common/typedefs.hpp>

namespace loco {

class HandStateDesired : public EndEffectorStateDesired, public HandStateBase {
 public:
  HandStateDesired() = delete;
  explicit HandStateDesired(unsigned int numFingers) : HandStateBase(numFingers) {}
  ~HandStateDesired() override = default;
};

} /* namespace loco */
