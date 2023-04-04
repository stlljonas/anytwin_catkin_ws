/*
 * WheelProperties.cpp
 *
 *  Created on: Dec 21, 2016
 *      Author: Gabriel Hottiger
 */

#include "loco/common/end_effectors/WheelProperties.hpp"

namespace loco {

WheelProperties::WheelProperties() : EndEffectorProperties(), diameter_(0.0) {}

double WheelProperties::getDiameter() const {
  return diameter_;
}

void WheelProperties::setDiameter(double diameter) {
  diameter_ = diameter;
}

} /* namespace loco */
