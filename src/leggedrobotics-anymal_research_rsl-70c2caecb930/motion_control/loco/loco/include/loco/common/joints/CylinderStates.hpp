/*
 * CylinderStates.hpp
 *
 *  Created on: Feb 21, 2017
 *      Author: Dominic Jud
 */

#pragma once

#include "loco/common/typedefs.hpp"

namespace loco {

class CylinderStates {
 public:
  explicit CylinderStates(const unsigned int nCylinders);
  virtual ~CylinderStates() = default;

  virtual const CylinderControlModes& getCylinderControlModes() const;
  virtual void setCylinderControlModes(const CylinderControlModes& cylinderControlModes);
  virtual void setCylinderControlMode(const unsigned int index, const CylinderControlMode& cylinderControlMode);

  virtual const CylinderPositions& getCylinderPositions() const;
  virtual void setCylinderPositions(const CylinderPositions& cylinderPositions);
  virtual void setCylinderPosition(const unsigned int index, const CylinderPosition& cylinderPosition);

  virtual const CylinderVelocities& getCylinderVelocities() const;
  virtual void setCylinderVelocities(const CylinderVelocities& cylinderVelocities);
  virtual void setCylinderVelocity(const unsigned int index, const CylinderVelocity& cylinderVelocity);

  virtual const CylinderForces& getCylinderForces() const;
  virtual void setCylinderForces(const CylinderForces& cylinderForces);
  virtual void setCylinderForce(const unsigned int index, const CylinderForce& cylinderForce);

 protected:
  //! The number of cylinders represented by this container
  const unsigned int nCylinders_;
  //! The vector of cylinder positions.
  CylinderPositions cylinderPositions_;
  //! The vector of cylinder velocities.
  CylinderVelocities cylinderVelocities_;
  //! The vector of cylinder torques.
  CylinderForces cylinderForces_;
  //! The vector of cylinder control modes.
  CylinderControlModes cylinderControlModes_;
};

} /* namespace loco */
