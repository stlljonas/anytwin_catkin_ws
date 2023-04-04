/*
 * MeasuredCylinderStates.hpp
 *
 *  Created on: Feb 21, 2017
 *      Author: Dominic Jud
 */

#pragma once

#include "loco/common/joints/CylinderStates.hpp"

namespace loco {

class MeasuredCylinderStates : public CylinderStates {
 public:
  explicit MeasuredCylinderStates(const unsigned int nCylinders);
  ~MeasuredCylinderStates() override = default;

  virtual const CylinderForces& getGravityCylinderForces() const;
  virtual void setGravityCylinderForces(const CylinderForces& gravityCylinderForces);
  virtual void setGravityCylinderForce(const unsigned int index, const CylinderForce& gravityCylinderForce);

 protected:
  //! The vector of joint torques due to gravity.
  CylinderForces gravityCylinderForces_;
};

} /* namespace loco */
