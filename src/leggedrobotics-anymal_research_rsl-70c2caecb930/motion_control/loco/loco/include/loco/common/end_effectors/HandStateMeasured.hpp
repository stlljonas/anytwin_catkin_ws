/*
 * HandStateMeasured.hpp
 *
 *  Created on: June 25, 2018
 *      Author: Markus Staeuble
 */

#pragma once

// loco
#include <loco/common/end_effectors/EndEffectorStateMeasured.hpp>
#include <loco/common/end_effectors/HandStateBase.hpp>
#include <loco/common/typedefs.hpp>

namespace loco {

class HandStateMeasured : public EndEffectorStateMeasured, public HandStateBase {
 public:
  HandStateMeasured() = delete;
  explicit HandStateMeasured(unsigned int numFingers) : HandStateBase(numFingers) {}
  ~HandStateMeasured() override = default;
};

} /* namespace loco */
