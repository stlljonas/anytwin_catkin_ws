/*
 * MotionPlanAngular.hpp
 *
 *  Created on: 07.08, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// motion generation utils
#include "motion_generation_utils/MotionPlanBase.hpp"

namespace motion_generation {

class MotionPlanAngular : virtual public MotionPlanBase {
public:
  MotionPlanAngular();
  ~MotionPlanAngular() override = default;

  EulerAnglesZyx getAnglesZyxInitialBaseToPlane() const;
  EulerAnglesZyxDiff getInitialAngularRatesZyxInPlaneFrame() const;
  EulerAnglesZyxDiff getInitialAngularAccelerationZyxInPlaneFrame() const;

  EulerAnglesZyx getAnglesZyxFinalBaseToPlane() const;
  EulerAnglesZyxDiff getFinalAngularRatesZyxInPlaneFrame() const;
  EulerAnglesZyxDiff getFinalAngularAccelerationZyxInPlaneFrame() const;

  void setAnglesZyxInitialBaseToPlane(const EulerAnglesZyx& initialStateInPlaneFrame);
  void setInitialAngularRatesZyxInPlaneFrame(const EulerAnglesZyxDiff& initialStateInPlaneFrame);
  void setInitialAngularAccelerationZyxInPlaneFrame(const EulerAnglesZyxDiff& initialStateInPlaneFrame);
  void setAnglesZyxFinalBaseToPlane(const EulerAnglesZyx& finalStateInPlaneFrame);
  void setFinalAngularRatesZyxInPlaneFrame(const EulerAnglesZyxDiff& finalStateInPlaneFrame);
  void setFinalAngularAccelerationZyxInPlaneFrame(const EulerAnglesZyxDiff& finalStateInPlaneFrame);

};

} /* namespace motion_generation */
