/*
 * WholeBodyProperties.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: Dario Bellicoso
 */

#include <loco/common/WholeBodyProperties.hpp>

namespace loco {

WholeBodyProperties::WholeBodyProperties() : totalMass_(0.0) {}

void WholeBodyProperties::setTotalMass(double totalMass) {
  totalMass_ = totalMass;
}

double WholeBodyProperties::getTotalMass() const {
  return totalMass_;
}

} /* namespace loco */
