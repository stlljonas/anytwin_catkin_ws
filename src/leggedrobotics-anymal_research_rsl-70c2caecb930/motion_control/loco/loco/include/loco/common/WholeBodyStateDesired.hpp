/*
 * WholeBodyStateDesired.hpp
 *
 *  Created on: Feb 19, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"

namespace loco {

class WholeBodyStateDesired {
 public:
  WholeBodyStateDesired();
  virtual ~WholeBodyStateDesired() = default;

  virtual const Position& getPositionWorldToWholeBodyCenterOfMassInWorldFrame() const;
  virtual void setPositionWorldToWholeBodyCenterOfMassInWorldFrame(const Position& positionWorldToWholeBodyCenterOfMassInWorldFrame);

  virtual const LinearVelocity& getLinearVelocityWholeBodyCenterOfMassInWorldFrame() const;
  virtual void setLinearVelocityWholeBodyCenterOfMassInWorldFrame(const LinearVelocity& linearVelocityWholeBodyCenterOfMassInWorldFrame);

  virtual const LinearAcceleration& getLinearAccelerationWholeBodyCenterOfMassInWorldFrame() const;
  virtual void setLinearAccelerationWholeBodyCenterOfMassInWorldFrame(
      const LinearAcceleration& linearAccelerationWholeBodyCenterOfMassInWorldFrame);

 protected:
  Position positionWorldToWholeBodyCenterOfMassInWorldFrame_;
  LinearVelocity linearVelocityWholeBodyCenterOfMassInWorldFrame_;
  LinearAcceleration linearAccelerationWholeBodyCenterOfMassInWorldFrame_;
};

} /* namespace loco */
