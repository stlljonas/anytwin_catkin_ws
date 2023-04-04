/*
 * FootBaseStateDesired.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/end_effectors/EndEffectorStateDesired.hpp>
#include <loco/common/end_effectors/FootBaseStateBase.hpp>
#include <loco/common/typedefs.hpp>

namespace loco {

class FootBaseStateDesired : public EndEffectorStateDesired, public FootBaseStateBase {
 public:
  FootBaseStateDesired() = default;
  ~FootBaseStateDesired() override = default;
};

} /* namespace loco */
