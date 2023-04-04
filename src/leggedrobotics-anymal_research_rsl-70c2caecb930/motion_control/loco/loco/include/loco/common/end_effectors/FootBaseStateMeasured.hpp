/*
 * FootBaseStateMeasured.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/end_effectors/EndEffectorStateMeasured.hpp>
#include <loco/common/end_effectors/FootBaseStateBase.hpp>
#include <loco/common/typedefs.hpp>

namespace loco {

class FootBaseStateMeasured : public EndEffectorStateMeasured, public FootBaseStateBase {
 public:
  FootBaseStateMeasured() = default;
  ~FootBaseStateMeasured() override = default;
};

} /* namespace loco */
