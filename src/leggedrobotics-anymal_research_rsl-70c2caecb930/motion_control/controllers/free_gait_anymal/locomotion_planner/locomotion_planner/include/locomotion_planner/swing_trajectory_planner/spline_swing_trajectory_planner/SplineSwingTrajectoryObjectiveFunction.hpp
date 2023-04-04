/*
 * SplineSwingTrajectoryObjectiveFunction.hpp
 *
 *  Created on: Sep 11, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "locomotion_planner/common/type_defs.hpp"
#include "locomotion_planner/common/Parameters.hpp"

#include <curves/CubicHermiteE3Curve.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <numopt_common/NonlinearObjectiveFunction.hpp>

namespace locomotion_planner {

class SplineSwingTrajectoryObjectiveFunction : public numopt_common::NonlinearObjectiveFunction
{
 public:

  SplineSwingTrajectoryObjectiveFunction(const grid_map::GridMap& elevationMap,
                                         const grid_map::SignedDistanceField& signedDistanceField,
                                         const Parameters& parameters);
  virtual ~SplineSwingTrajectoryObjectiveFunction();

  void setInitialSolution(std::vector<curves::CubicHermiteE3Curve::ValueType> knotPositions);

  void setTrajectoryParameters(const Position& startPosition,
                               const Position& targetPosition,
                               const LinearVelocity& liftOffVelocity,
                               const LinearVelocity& touchdownVelocity,
                               const double averageVelocity,
                               const double minimumDuration);

  bool computeValue(numopt_common::Scalar& value, const numopt_common::Parameterization& params,
                    bool newParams = true);

  bool getLocalGradient(numopt_common::Vector& gradient, const numopt_common::Parameterization& params,
                        bool newParams = true);

  bool getLocalHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params,
                       bool newParams = true);

 private:

  double computeCollisionFactor(const Position& position, const Position& referencePosition,
                                const double collisionDepth) const;

  const Parameters& parameters_;
  const grid_map::GridMap& elevationMap_;
  const grid_map::SignedDistanceField& signedDistanceField_;
  std::vector<curves::CubicHermiteE3Curve::ValueType> initialKnotPositions_;
  Position startPosition_;
  Position targetPosition_;
  LinearVelocity liftOffVelocity_;
  LinearVelocity touchdownVelocity_;
  double averageVelocity_;
  double minimumDuration_;
  double startCollisionDepth_;
  double targetCollisionDepth_;
};

} /* namespace free_gait */
