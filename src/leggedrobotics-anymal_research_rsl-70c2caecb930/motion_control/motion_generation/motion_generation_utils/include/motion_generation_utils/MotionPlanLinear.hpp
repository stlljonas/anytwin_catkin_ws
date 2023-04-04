/*
 * MotionPlanLinear.hpp
 *
 *  Created on: 07.08, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// motion generation utils
#include "motion_generation_utils/MotionPlanBase.hpp"

namespace motion_generation {

class MotionPlanLinear : virtual public MotionPlanBase {
public:
  MotionPlanLinear();
  ~MotionPlanLinear() override = default;

  Position getPlaneToInitialPositionInPlaneFrame() const;
  LinearVelocity getInitialVelocityInPlaneFrame() const;
  LinearAcceleration getInitialAccelerationInPlaneFrame() const;

  Position getPlaneToFinalPositionInPlaneFrame() const;
  LinearVelocity getFinalVelocityInPlaneFrame() const;
  LinearAcceleration getFinalAccelerationInPlaneFrame() const;

  void getWorldToInitialPositionInWorldFrame(Position& translationalStateInWorldFrame) const;
  void getWorldToFinalPositionInWorldFrame(Position& translationalStateInWorldFrame) const;
  void getInitialVelocityInWorldFrame(LinearVelocity& translationalStateInWorldFrame) const;
  void getFinalVelocityInWorldFrame(LinearVelocity& translationalStateInWorldFrame) const;
  void getInitialAccelerationInWorldFrame(LinearAcceleration& translationalStateInWorldFrame) const;
  void getFinalAccelerationInWorldFrame(LinearAcceleration& translationalStateInWorldFrame) const;

  void setPlaneToInitialPositionInPlaneFrame(const Position& initialStateInPlaneFrame);
  void setPlaneToFinalPositionInPlaneFrame(const Position& finalStateInPlaneFrame);
  void setInitialVelocityInPlaneFrame(const LinearVelocity& initialStateInPlaneFrame);
  void setFinalVelocityInPlaneFrame(const LinearVelocity& finalStateInPlaneFrame);
  void setInitialAccelerationInPlaneFrame(const LinearAcceleration& initialStateInPlaneFrame);
  void setFinalAccelerationInPlaneFrame(const LinearAcceleration& finalStateInPlaneFrame);
};

} /* namespace motion_generation */
