/*
 * TrajectoryStateHandlerAngular.hpp
 *
 *  Created on: 07.09, 2018
 *      Author: Fabian Jenelten
 */


#pragma once

//zmp
#include "motion_generation_utils/TrajectoryStateHandlerBase.hpp"


namespace motion_generation {

class TrajectoryStateHandlerAngular : virtual public TrajectoryStateHandlerBase {
public:
  TrajectoryStateHandlerAngular();
  ~TrajectoryStateHandlerAngular() override = default;

  EulerAnglesZyx getAnglesZyxBaseToPlane() const;
  EulerAnglesZyx getAnglesZyxBaseToPlaneAtTime(const double tk) const;

  EulerAnglesZyxDiff getEulerRatesZyxBaseInPlaneFrame() const;
  EulerAnglesZyxDiff getEulerRatesZyxBaseInPlaneFrameAtTime(const double tk) const;

  EulerAnglesZyxDiff getEulerAccelerationZyxBaseInPlaneFrame() const;
  EulerAnglesZyxDiff getEulerAccelerationZyxBaseInPlaneFrameAtTime(const double tk) const;

  LocalAngularVelocity getAngularVelocityBaseInPlaneFrame() const;
  LocalAngularVelocity getAngularVelocityBaseInPlaneFrameAtTime(const double tk) const;

  AngularAcceleration getAngularAccelerationBaseInPlaneFrame() const;
  AngularAcceleration getAngularAccelerationBaseInPlaneFrameAtTime(const double tk) const;

protected:
  //! Add motion plan (angular part) using knot points.
  bool addMotionPlanOrientation(
      const eulerAnglesZyxVector& anglesZyxPathToPlane,
      const eulerAnglesZyxDiffVector& angularRatesZyxPathInPlaneFrame,
      const std::vector<double>& duration,
      double optimizationHorizonInSeconds);

};

} /* motion_generation */
