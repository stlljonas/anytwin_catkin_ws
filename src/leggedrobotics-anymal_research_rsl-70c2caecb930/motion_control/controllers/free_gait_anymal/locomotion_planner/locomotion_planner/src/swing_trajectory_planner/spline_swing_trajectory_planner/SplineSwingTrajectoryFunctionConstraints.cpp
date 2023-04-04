/*
 * SplineSwingTrajectoryFunctionConstraints.cpp
 *
 *  Created on: Sep 11, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "locomotion_planner/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryFunctionConstraints.hpp"

namespace locomotion_planner {

SplineSwingTrajectoryFunctionConstraints::SplineSwingTrajectoryFunctionConstraints(const size_t nKnots, const Parameters& parameters)
    : NonlinearFunctionConstraints(),
      parameters_(parameters),
      nKnots_(nKnots)
{
  inequalityConstraintMinValues_ = Eigen::VectorXd::Constant(nKnots_ * nDimensionsPerKnot_, std::numeric_limits<double>::lowest());
  inequalityConstraintMaxValues_ = Eigen::VectorXd::Constant(nKnots_ * nDimensionsPerKnot_, std::numeric_limits<double>::max());
}

SplineSwingTrajectoryFunctionConstraints::~SplineSwingTrajectoryFunctionConstraints()
{
}

void SplineSwingTrajectoryFunctionConstraints::setTrajectoryParameters(const Position& startPosition,
                                                                       const Position& targetPosition)
{
  startPosition_ = startPosition;
  targetPosition_ = targetPosition;
}

void SplineSwingTrajectoryFunctionConstraints::setInitialSolution(std::vector<curves::CubicHermiteE3Curve::ValueType> knotPositions)
{
  inequalityConstraintMinValues_ = Eigen::VectorXd::Constant(nKnots_ * nDimensionsPerKnot_, std::numeric_limits<double>::lowest());
  inequalityConstraintMaxValues_ = Eigen::VectorXd::Constant(nKnots_ * nDimensionsPerKnot_, std::numeric_limits<double>::max());
  for (size_t i = 0; i < nKnots_; ++i) {
    inequalityConstraintMaxValues_(i * nDimensionsPerKnot_ + 2) =
        (startPosition_.z() + targetPosition_.z()) / 2.0 + parameters_.getSwingTrajectoryMaxHeight();
    inequalityConstraintMinValues_(i * nDimensionsPerKnot_ + 2) = parameters_.getSwingTrajectoryMinClearance();
  }
}

bool SplineSwingTrajectoryFunctionConstraints::getGlobalBoundConstraintMinValues(
    numopt_common::Vector& values)
{
  values = inequalityConstraintMinValues_;
  return true;
}

bool SplineSwingTrajectoryFunctionConstraints::getGlobalBoundConstraintMaxValues(
    numopt_common::Vector& values)
{
  values = inequalityConstraintMaxValues_;
  return true;
}

bool SplineSwingTrajectoryFunctionConstraints::getInequalityConstraintValues(numopt_common::Vector& values,
                                                                             const numopt_common::Parameterization& p,
                                                                             bool newParams)
{
  values = p.getParams();
  return true;
}

bool SplineSwingTrajectoryFunctionConstraints::getInequalityConstraintMinValues(numopt_common::Vector& d)
{
  d.resize(0);
  return true;
}

bool SplineSwingTrajectoryFunctionConstraints::getInequalityConstraintMaxValues(numopt_common::Vector& f)
{
  f.resize(0);
  return true;
}

} /* namespace */
