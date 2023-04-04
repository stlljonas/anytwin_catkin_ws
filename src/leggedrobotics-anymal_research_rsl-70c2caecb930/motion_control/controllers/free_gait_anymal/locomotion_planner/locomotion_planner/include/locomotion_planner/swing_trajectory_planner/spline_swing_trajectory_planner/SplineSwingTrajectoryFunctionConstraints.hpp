/*
 * SplineSwingTrajectoryFunctionConstraints.hpp
 *
 *  Created on: Sep 11, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "locomotion_planner/common/Parameters.hpp"

#include <numopt_common/NonlinearFunctionConstraints.hpp>
#include <curves/CubicHermiteE3Curve.hpp>

namespace locomotion_planner {

class SplineSwingTrajectoryFunctionConstraints : public numopt_common::NonlinearFunctionConstraints
{
 public:

  SplineSwingTrajectoryFunctionConstraints(const size_t nKnots, const Parameters& parameters);
  virtual ~SplineSwingTrajectoryFunctionConstraints();

  void setTrajectoryParameters(const Position& startPosition, const Position& targetPosition);
  void setInitialSolution(std::vector<curves::CubicHermiteE3Curve::ValueType> knotPositions);

  bool getGlobalBoundConstraintMinValues(numopt_common::Vector& values);
  bool getGlobalBoundConstraintMaxValues(numopt_common::Vector& values);

  //! d <= c(p) <= f
  bool getInequalityConstraintValues(numopt_common::Vector& values,
                                     const numopt_common::Parameterization& p,
                                     bool newParams = true);
  bool getInequalityConstraintMinValues(numopt_common::Vector& d);
  bool getInequalityConstraintMaxValues(numopt_common::Vector& f);

 private:
  const Parameters& parameters_;
  static const size_t nDimensionsPerKnot_ = 3;
  const size_t nKnots_;
  numopt_common::Vector inequalityConstraintMinValues_;
  numopt_common::Vector inequalityConstraintMaxValues_;
  Position startPosition_;
  Position targetPosition_;
};

} /* namespace */
