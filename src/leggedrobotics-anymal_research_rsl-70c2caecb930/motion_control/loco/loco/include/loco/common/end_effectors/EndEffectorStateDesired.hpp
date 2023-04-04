/*
 * EndEffectorStateDesired.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/end_effectors/EndEffectorStateBase.hpp>
#include <loco/common/joints/DesiredJointStates.hpp>
#include <loco/common/typedefs.hpp>

// STL
#include <memory>

namespace loco {

class EndEffectorStateDesired : public EndEffectorStateBase {
 public:
  EndEffectorStateDesired();
  ~EndEffectorStateDesired() override = default;

  void setDesiredJointStatesLimb(DesiredJointStates* const desiredJointStatesLimb);

 protected:
  DesiredJointStates* desiredJointStatesLimb_;
};

using EndEffectorStateDesiredPtr = std::unique_ptr<EndEffectorStateDesired>;

} /* namespace loco */
