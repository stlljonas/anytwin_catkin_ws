/*
 * CylinderStates.cpp
 *
 *  Created on: Feb 21, 2017
 *      Author: Dominic Jud
 */

#include "loco/common/joints/CylinderStates.hpp"

// STL
#include <cassert>

namespace loco {

CylinderStates::CylinderStates(const unsigned int nCylinders) : nCylinders_(nCylinders) {
  cylinderControlModes_.resize(nCylinders_, 1);
  cylinderControlModes_.setZero();
  cylinderPositions_.resize(nCylinders_, 1);
  cylinderPositions_.setZero();
  cylinderVelocities_.resize(nCylinders_, 1);
  cylinderVelocities_.setZero();
  cylinderForces_.resize(nCylinders_, 1);
  cylinderForces_.setZero();
}

const CylinderControlModes& CylinderStates::getCylinderControlModes() const {
  return cylinderControlModes_;
}

void CylinderStates::setCylinderControlModes(const CylinderControlModes& cylinderControlModes) {
  assert(cylinderControlModes.rows() == cylinderControlModes_.rows());
  cylinderControlModes_ = cylinderControlModes;
}

void CylinderStates::setCylinderControlMode(const unsigned int index, const CylinderControlMode& cylinderControlMode) {
  assert(index < cylinderControlModes_.rows());
  cylinderControlModes_(index) = cylinderControlMode;
}

const CylinderPositions& CylinderStates::getCylinderPositions() const {
  return cylinderPositions_;
}

void CylinderStates::setCylinderPositions(const CylinderPositions& cylinderPositions) {
  assert(cylinderPositions.rows() == cylinderPositions_.rows());
  cylinderPositions_ = cylinderPositions;
}

void CylinderStates::setCylinderPosition(const unsigned int index, const CylinderPosition& cylinderPosition) {
  assert(index < cylinderPositions_.rows());
  cylinderPositions_(index) = cylinderPosition;
}

const CylinderVelocities& CylinderStates::getCylinderVelocities() const {
  return cylinderVelocities_;
}

void CylinderStates::setCylinderVelocities(const CylinderVelocities& cylinderVelocities) {
  assert(cylinderVelocities.rows() == cylinderVelocities_.rows());
  cylinderVelocities_ = cylinderVelocities;
}

void CylinderStates::setCylinderVelocity(const unsigned int index, const CylinderVelocity& cylinderVelocity) {
  assert(index < cylinderVelocities_.rows());
  cylinderVelocities_(index) = cylinderVelocity;
}

const CylinderForces& CylinderStates::getCylinderForces() const {
  return cylinderForces_;
}

void CylinderStates::setCylinderForces(const CylinderForces& cylinderForces) {
  assert(cylinderForces.rows() == cylinderForces_.rows());
  cylinderForces_ = cylinderForces;
}

void CylinderStates::setCylinderForce(const unsigned int index, const CylinderForce& cylinderForce) {
  assert(index < cylinderForces_.rows());
  cylinderForces_(index) = cylinderForce;
}

} /* namespace loco */
