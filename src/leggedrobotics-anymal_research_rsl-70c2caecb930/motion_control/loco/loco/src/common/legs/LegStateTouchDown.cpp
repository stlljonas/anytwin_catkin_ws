/*
 * LegStateTouchDown.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

// loco
#include "loco/common/legs/LegStateTouchDown.hpp"

namespace loco {

LegStateTouchDown::LegStateTouchDown() : LegStateBase(), positionWorldToFootInWorldFrame_() {}

void LegStateTouchDown::setPositionWorldToFootInWorldFrame(const loco::Position& positionWorldToFootInWorldFrame) {
  positionWorldToFootInWorldFrame_ = positionWorldToFootInWorldFrame;
}

const Position& LegStateTouchDown::getPositionWorldToFootInWorldFrame() const {
  return positionWorldToFootInWorldFrame_;
}

} /* namespace loco */