/*
 * LegProperties.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Christian Gehring, Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// loco
#include "loco/common/legs/LegProperties.hpp"

namespace loco {

const Position& LegProperties::getDesiredDefaultSteppingPositionHipToFootInControlFrame() const {
  return desiredDefaultSteppingPositionHipToFootInControlFrame_;
}

void LegProperties::setDesiredDefaultSteppingPositionHipToFootInControlFrame(const Position& position) {
  desiredDefaultSteppingPositionHipToFootInControlFrame_ = position;
}

} /* namespace loco */
