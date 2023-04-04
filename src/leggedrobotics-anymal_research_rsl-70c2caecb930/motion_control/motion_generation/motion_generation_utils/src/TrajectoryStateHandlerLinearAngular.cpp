/*
 * TrajectoryStateHandlerLinearAngular.cpp
 *
 *  Created on: 07.09, 2018
 *      Author: Fabian Jenelten
 */

// motion generation utils
#include "motion_generation_utils/TrajectoryStateHandlerLinearAngular.hpp"


namespace motion_generation {

TrajectoryStateHandlerLinearAngular::TrajectoryStateHandlerLinearAngular():
  TrajectoryStateHandlerLinear(),
  TrajectoryStateHandlerAngular()
{
  optimizationState_.clear();
  optimizationState_.reserve(zmp::optimizationTranslationalRotationalDofs.size());
  for(const auto& dim : zmp::optimizationTranslationalRotationalDofs) {
    optimizationState_.push_back(dim);
  }
}


bool TrajectoryStateHandlerLinearAngular::setMotionPlanLinearAngular(
    const positionVector& positionKnotInPlaneFrame,
    const velocityVector& velocityKnotInPlaneFrame,
    const accelerationVector& accelerationKnotInPlaneFrame,
    const eulerAnglesZyxVector& anglesZyxPathToPlane,
    const eulerAnglesZyxDiffVector& angularRatesZyxPathInPlaneFrame,
    const std::vector<double>& duration,
    double optimizationHorizonInSeconds) {
  // Clear all splines.
  if(!reset()) { return false; }

  // Add splines for translational part.
  if(!addMotionPlanTranslational(
      positionKnotInPlaneFrame,
      velocityKnotInPlaneFrame,
      accelerationKnotInPlaneFrame,
      duration,
      optimizationHorizonInSeconds)) { return false; }

  // Add spline for rotational part.
  if(!addMotionPlanOrientation(
      anglesZyxPathToPlane,
      angularRatesZyxPathInPlaneFrame,
      duration,
      optimizationHorizonInSeconds)) { return false; }

  // Initialize container.
  setContainerDuration(optimizationHorizonInSeconds);
  if(!setContainerTime(0.0)) { return false; }
  return true;
}

} /* namespace motion_generation */
