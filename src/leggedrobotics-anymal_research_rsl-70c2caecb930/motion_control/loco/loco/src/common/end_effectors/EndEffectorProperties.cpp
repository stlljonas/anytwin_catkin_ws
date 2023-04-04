/*
 * EndEffectorProperties.cpp
 *
 *  Created on: Dec 21, 2016
 *      Author: Gabriel Hottiger, Dario Bellicoso
 */

#include "loco/common/end_effectors/EndEffectorProperties.hpp"

namespace loco {

EndEffectorProperties::EndEffectorProperties() : mass_(0.0), positionBaseToCenterOfMassInBaseFrame_(), numberOfContactConstraints_(3) {}

EndEffectorProperties::EndEffectorProperties(unsigned int numberOfContactConstraints)
    : mass_(0.0), positionBaseToCenterOfMassInBaseFrame_(), numberOfContactConstraints_(numberOfContactConstraints) {}

unsigned int EndEffectorProperties::getNumberOfContactConstraints() const {
  return numberOfContactConstraints_;
}

} /* namespace loco */
