/*
 * SplineSwingTrajectoryObjectiveFunction.cpp
 *
 *  Created on: Sep 11, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "locomotion_planner/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryObjectiveFunction.hpp"
#include "locomotion_planner/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryParameterization.hpp"

#include <message_logger/message_logger.hpp>

namespace locomotion_planner {

SplineSwingTrajectoryObjectiveFunction::SplineSwingTrajectoryObjectiveFunction(
    const grid_map::GridMap& elevationMap, const grid_map::SignedDistanceField& signedDistanceField,
    const Parameters& parameters)
    : NonlinearObjectiveFunction(),
      parameters_(parameters),
      elevationMap_(elevationMap),
      signedDistanceField_(signedDistanceField),
      averageVelocity_(0.0),
      minimumDuration_(0.0),
      startCollisionDepth_(0.0),
      targetCollisionDepth_(0.0)
{

}

SplineSwingTrajectoryObjectiveFunction::~SplineSwingTrajectoryObjectiveFunction()
{
}

void SplineSwingTrajectoryObjectiveFunction::setInitialSolution(std::vector<curves::CubicHermiteE3Curve::ValueType> knotPositions)
{
  initialKnotPositions_ = knotPositions;
}

void SplineSwingTrajectoryObjectiveFunction::setTrajectoryParameters(const Position& startPosition,
                                                                     const Position& targetPosition,
                                                                     const LinearVelocity& liftOffVelocity,
                                                                     const LinearVelocity& touchdownVelocity,
                                                                     const double averageVelocity,
                                                                     const double minimumDuration)
{
  startPosition_ = startPosition;
  targetPosition_ = targetPosition;
  liftOffVelocity_ = liftOffVelocity;
  touchdownVelocity_ = touchdownVelocity;
  averageVelocity_ = averageVelocity;
  minimumDuration_ = minimumDuration;
  startCollisionDepth_ = elevationMap_.atPosition(parameters_.getCollisionLayer(), startPosition.vector().head(2)) - startPosition.z();
  startCollisionDepth_ = std::max(startCollisionDepth_, parameters_.getFootCenterHeight());
  targetCollisionDepth_ = elevationMap_.atPosition(parameters_.getCollisionLayer(), targetPosition.vector().head(2)) - targetPosition.z();
  targetCollisionDepth_ = std::max(targetCollisionDepth_, parameters_.getFootCenterHeight());
}

bool SplineSwingTrajectoryObjectiveFunction::computeValue(numopt_common::Scalar& value,
                                                          const numopt_common::Parameterization& params, bool newParams)
{
  SplineSwingTrajectoryParameterization splineParameterization(params); // Need a copy.
  splineParameterization.computeTrajectory(startPosition_, targetPosition_, liftOffVelocity_, touchdownVelocity_,
                                           averageVelocity_, minimumDuration_);
  std::map<double, std::tuple<Position, LinearVelocity>> splinePoints;
  splineParameterization.getSampledValues(static_cast<size_t>(5), splinePoints);

  // Integrate through spline.
  double collisionCosts = 0.0;
  double lengthCosts = 0.0;
  const double duration = splinePoints.rbegin()->first; // We assume time starts at zero.
  for (auto splineIt = splinePoints.begin(); splineIt != splinePoints.end(); ++splineIt) {
    const double time = (*splineIt).first;
    const std::tuple<Position, LinearVelocity>& splinePoint = (*splineIt).second;
    const Position& position = std::get<0>(splinePoint);
    const LinearVelocity& velocity = std::get<1>(splinePoint);

    // Length Costs
    if (splineIt != --splinePoints.end()) {
      const Position& netPosition = std::get<0>((*std::next(splineIt, 1)).second);
      lengthCosts += (position - netPosition).norm();
    }

    // Collision Costs
    const double phase = time / duration;
    double factor = 1.0;
    if (phase < 0.5 && std::isfinite(startCollisionDepth_)) {
      factor = computeCollisionFactor(position, startPosition_, startCollisionDepth_);
    }
    else if (phase >= 0.5 && std::isfinite(targetCollisionDepth_)) {
      factor = computeCollisionFactor(position, targetPosition_, targetCollisionDepth_);
    }

    const double d = signedDistanceField_.getInterpolatedDistanceAt(position.vector());
    const double epsilon = 0.07; // See CHOMP Paper.
    double collisionCost = 0.0;
    if (d < 0.0) collisionCost = -d + 0.5 * epsilon;
    else if (0.0 <= d && d <= epsilon) collisionCost = 1.0 / (2.0 * epsilon) * (d - epsilon) * (d - epsilon);
    collisionCosts += velocity.norm() * collisionCost;
  }

  // Height costs (alternative to length).
  const double heightCosts = params.getParams()(2) - (startPosition_.z() + targetPosition_.z()) / 2.0;

  // Regularizer.
  double regularizerCosts = 0.0;
  std::vector<curves::CubicHermiteE3Curve::ValueType> knotPositions = splineParameterization.getKnotPositions();
  for (size_t i = 0; i < initialKnotPositions_.size(); ++i) {
    // TODO(Valentin Yuryev): Impirically tuned. Create parameters for this if necessary after tested over time.
    regularizerCosts += 3.0 * (initialKnotPositions_[i] - knotPositions[i]).head(2).norm();
  }

  // TODO(Valentin Yuryev): Collision multiplication factor is empirically adjusted. Create parameters for this if necessary.
  value = lengthCosts + heightCosts + 5.0 * collisionCosts + regularizerCosts * regularizerCosts;
  return true;
}

bool SplineSwingTrajectoryObjectiveFunction::getLocalGradient(numopt_common::Vector& gradient,
                                                         const numopt_common::Parameterization& params, bool newParams)
{
  return NonlinearObjectiveFunction::estimateLocalGradient(gradient, params, 1.0e-3);
}

bool SplineSwingTrajectoryObjectiveFunction::getLocalHessian(numopt_common::SparseMatrix& hessian,
                                                             const numopt_common::Parameterization& params,
                                                             bool newParams)
{
  return NonlinearObjectiveFunction::estimateLocalHessian(hessian, params, 1.0e-3);
}

double SplineSwingTrajectoryObjectiveFunction::computeCollisionFactor(const Position& position, const Position& referencePosition,
                              const double collisionDepth) const
{
  // https://math.stackexchange.com/questions/417717/looking-for-a-3d-smooth-step-function
  const double deadZoneRadius = 0.05;
  const double planarDistance = (position.vector().head(2) - referencePosition.vector().head(2)).norm() - deadZoneRadius;
  const double factor = 0.5 * (1.0 + tanh(planarDistance / collisionDepth));
  return factor * factor;
}

} /* namespace free_gait */

