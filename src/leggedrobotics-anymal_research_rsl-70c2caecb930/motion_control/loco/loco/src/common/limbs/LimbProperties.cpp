/*
 * LimbProperties.cpp
 *
 *  Created on: Dec 21, 2016
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/common/limbs/LimbProperties.hpp"

namespace loco {

LimbProperties::LimbProperties() : limbMass_(0.0), positionBaseToLimbComInBaseFrame_() {}

double LimbProperties::getLimbMass() const {
  return limbMass_;
}

void LimbProperties::setLimbMass(double limbMass) {
  limbMass_ = limbMass;
}

const Position& LimbProperties::getPositionBaseToLimbComInBaseFrame() const {
  return positionBaseToLimbComInBaseFrame_;
}

void LimbProperties::setPositionBaseToLimbComInBaseFrame(const Position& positionBaseToLimbComInBaseFrame) {
  positionBaseToLimbComInBaseFrame_ = positionBaseToLimbComInBaseFrame;
}

} /* namespace loco */
