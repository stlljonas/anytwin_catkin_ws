/*
 * MeasuredCylinderStates.cpp
 *
 *  Created on: Feb 21, 2017
 *      Author: Dominic Jud
 */

#include "loco/common/joints/MeasuredCylinderStates.hpp"

// STL
#include <cassert>

namespace loco {

MeasuredCylinderStates::MeasuredCylinderStates(const unsigned int nCylinders) : CylinderStates(nCylinders) {
  gravityCylinderForces_.resize(nCylinders_, 1);
  gravityCylinderForces_.setZero();
}

const JointTorques& MeasuredCylinderStates::getGravityCylinderForces() const {
  return gravityCylinderForces_;
}

void MeasuredCylinderStates::setGravityCylinderForces(const CylinderForces& gravityCylinderForces) {
  assert(gravityCylinderForces.rows() == gravityCylinderForces_.rows());
  gravityCylinderForces_ = gravityCylinderForces;
}

void MeasuredCylinderStates::setGravityCylinderForce(const unsigned int index, const CylinderForce& gravityCylinderForce) {
  assert(index < gravityCylinderForces_.rows());
  gravityCylinderForces_(index) = gravityCylinderForce;
}

} /* namespace loco */
