/*
 * LegStateLiftOff.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

// loco
#include "loco/common/legs/LegStateLiftOff.hpp"

namespace loco {

LegStateLiftOff::LegStateLiftOff()
    : LegStateBase(),
      positionWorldToFootInWorldFrame_(),
      positionWorldToHipInWorldFrame_(),
      positionWorldToHipOnTerrainAlongWorldZAtLiftOffInWorldFrame_() {}

const Position& LegStateLiftOff::getPositionWorldToHipInWorldFrame() const {
  return positionWorldToHipInWorldFrame_;
}

const Position& LegStateLiftOff::getPositionWorldToFootInWorldFrame() const {
  return positionWorldToFootInWorldFrame_;
}

const Position& LegStateLiftOff::getPositionWorldToHipOnTerrainAlongWorldZAtLiftOffInWorldFrame() const {
  return positionWorldToHipOnTerrainAlongWorldZAtLiftOffInWorldFrame_;
}

void LegStateLiftOff::setPositionWorldToHipInWorldFrame(const Position& positionWorldToHipInWorldFrame) {
  positionWorldToHipInWorldFrame_ = positionWorldToHipInWorldFrame;
}

void LegStateLiftOff::setPositionWorldToFootInWorldFrame(const Position& positionWorldToFootInWorldFrame) {
  positionWorldToFootInWorldFrame_ = positionWorldToFootInWorldFrame;
}

void LegStateLiftOff::setPositionWorldToHipOnTerrainAlongWorldZInWorldFrame(
    const Position& positionWorldToHipOnTerrainAlongWorldZAtLiftOffInWorldFrame) {
  positionWorldToHipOnTerrainAlongWorldZAtLiftOffInWorldFrame_ = positionWorldToHipOnTerrainAlongWorldZAtLiftOffInWorldFrame;
}

} /* namespace loco */