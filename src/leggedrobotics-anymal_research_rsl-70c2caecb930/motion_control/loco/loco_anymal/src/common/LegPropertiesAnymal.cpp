/*
 * LegPropertiesAnymal.cpp
 *
 *  Created on: Jan 16, 2018
 *      Author: Dario Bellicoso
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

// loco anymal
#include "loco_anymal/common/LegPropertiesAnymal.hpp"

namespace loco_anymal {

LegPropertiesAnymal::LegPropertiesAnymal(
    LimbEnum limb,
    const AnymalModel& model) :
      LegPropertiesRomo(limb, model),
      model_(model)
{ }

double LegPropertiesAnymal::getMaximumLimbExtension() const {
  return model_.getMaxHipToFootLength();
}

double LegPropertiesAnymal::getMinimumLimbExtension() const {
  return model_.getMinHipToFootLength();
}

} /* namespace loco_anymal */
