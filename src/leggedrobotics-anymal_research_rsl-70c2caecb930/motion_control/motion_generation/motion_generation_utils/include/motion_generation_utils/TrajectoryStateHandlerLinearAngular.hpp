/*
 * TrajectoryStateHandlerLinearAngular.hpp
 *
 *  Created on: 07.09, 2018
 *      Author: Fabian Jenelten
 *
 *
 *                  TrajectoryStateHandlerBase
 *                      /                 \
 *                     /                   \
 *  TrajectoryStateHandlerLinear      TrajectoryStateHandlerAngular
 *                   |                      |
 *                   |                      |
 *             TrajectoryStateHandlerLinearAngular
 */


#pragma once

//motion_generation_utils
#include "motion_generation_utils/TrajectoryStateHandlerLinear.hpp"
#include "motion_generation_utils/TrajectoryStateHandlerAngular.hpp"


namespace motion_generation {

class TrajectoryStateHandlerLinearAngular :
    virtual public TrajectoryStateHandlerLinear,
    virtual public TrajectoryStateHandlerAngular {
public:
  TrajectoryStateHandlerLinearAngular();

  ~TrajectoryStateHandlerLinearAngular() override = default;

  //! Set motion plan using knot points.
  bool setMotionPlanLinearAngular(
      const positionVector& positionKnotInPlaneFrame,
      const velocityVector& velocityKnotInPlaneFrame,
      const accelerationVector& accelerationKnotInPlaneFrame,
      const eulerAnglesZyxVector& anglesZyxPathToPlane,
      const eulerAnglesZyxDiffVector& angularRatesZyxPathInPlaneFrame,
      const std::vector<double>& duration,
      double optimizationHorizonInSeconds);

};

} /* namespace motion_generation */
