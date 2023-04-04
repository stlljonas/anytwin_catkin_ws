/*
 * HandStatebase.hpp
 *
 *  Created on: June 25, 2018
 *      Author: Markus Staeuble
 */

#pragma once

// loco
#include <loco/common/joints/FingerStates.hpp>
#include <loco/common/typedefs.hpp>

namespace loco {

class HandStateBase {
 public:
  HandStateBase() = delete;

  explicit HandStateBase(unsigned int numFingers) : fingerStates_(numFingers) {}

  virtual ~HandStateBase() = default;

  const FingerStates& getFingerStates() const { return fingerStates_; }

  FingerStates& getFingerStates() { return fingerStates_; }

 protected:
  FingerStates fingerStates_;
};

} /* namespace loco */
