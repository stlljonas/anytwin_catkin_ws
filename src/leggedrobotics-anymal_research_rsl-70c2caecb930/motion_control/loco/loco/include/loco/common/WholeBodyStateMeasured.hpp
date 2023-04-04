/*
 * WholeBodyStateMeasured.hpp
 *
 *  Created on: Feb 19, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"

namespace loco {

class WholeBodyStateMeasured {
 public:
  WholeBodyStateMeasured();
  virtual ~WholeBodyStateMeasured() = default;

  virtual const Position& getPositionWorldToWholeBodyCenterOfMassInWorldFrame() const;
  virtual void setPositionWorldToWholeBodyCenterOfMassInWorldFrame(const Position& positionWorldToWholeBodyCenterOfMassInWorldFrame);

  virtual const LinearVelocity& getLinearVelocityWholeBodyCenterOfMassInWorldFrame() const;
  virtual void setLinearVelocityWholeBodyCenterOfMassInWorldFrame(const LinearVelocity& linearVelocityWholeBodyCenterOfMassInWorldFrame);

  virtual const Position& getPositionControlToWholeBodyCenterOfMassInControlFrame() const;
  virtual void setPositionControlToWholeBodyCenterOfMassInControlFrame(
      const Position& positionControlToWholeBodyCenterOfMassInControlFrame);

  virtual const Position& getPositionWorldToCenterOfPressureInWorldFrame() const;
  virtual void setPositionWorldToCenterOfPressureInWorldFrame(const Position& positionWorldToCenterOfPressureInWorldFrame);

  virtual const LinearVelocity& getLinearVelocityWholeBodyCenterOfMassInControlFrame() const;
  virtual void setLinearVelocityWholeBodyCenterOfMassInControlFrame(
      const LinearVelocity& linearVelocityWholeBodyCenterOfMassInControlFrame);

  /*! Get position of the divergent component of motion.
   *
   * @return dcm Position of the DCM in the world frame.
   */
  virtual const Position& getPositionWorldToDCMInWorldFrame() const;

  /*! Update divergent component of motion based on the desired natural frequency omega.
   *
   * @param omega Natural frequency of desired dynamics.
   */
  virtual void updatePositionWorldToDCMInWorldFrame(double omega);

 protected:
  Position positionWorldToWholeBodyCenterOfMassInWorldFrame_;
  Position positionWorldToDCMInWorldFrame_;
  LinearVelocity linearVelocityWholeBodyCenterOfMassInWorldFrame_;
  LinearAcceleration linearAccelerationWholeBodyCenterOfMassInWorldFrame_;

  Position positionControlToWholeBodyCenterOfMassInControlFrame_;
  LinearVelocity linearVelocityWholeBodyCenterOfMassInControlFrame_;

  Position positionWorldToCenterOfPressureInWorldFrame_;
};

} /* namespace loco */
