/*
 * PoseOptimizationFunctionConstraints.hpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/executor/AdapterBase.hpp"
#include "free_gait_core/executor/State.hpp"
#include "free_gait_core/pose_optimization/PoseConstraintsChecker.hpp"

#include <numopt_common/NonlinearFunctionConstraints.hpp>
#include <grid_map_core/Polygon.hpp>

#include <map>

namespace free_gait {

class PoseOptimizationFunctionConstraints : public numopt_common::NonlinearFunctionConstraints
{
 public:
  using LimbLengths = PoseConstraintsChecker::LimbLengths;
  typedef std::map<LimbEnum, Position> LegPositions;

  PoseOptimizationFunctionConstraints();
  virtual ~PoseOptimizationFunctionConstraints();

  void setStance(const Stance& stance);
  void setSupportRegion(const grid_map::Polygon& supportRegion);

  void setLimbLengthConstraints(const LimbLengths& minLimbLenghts,
                                const LimbLengths& maxLimbLenghts);

  void setPositionsBaseToHip(const LegPositions& positionBaseToHipInBaseFrame);

  void setCenterOfMass(const Position& centerOfMassInBaseFrame);

  bool getGlobalBoundConstraintMinValues(numopt_common::Vector& values);
  bool getGlobalBoundConstraintMaxValues(numopt_common::Vector& values);

  //! d <= c(p) <= f
  bool getInequalityConstraintValues(numopt_common::Vector& values,
                                     const numopt_common::Parameterization& p,
                                     bool newParams = true);
  bool getInequalityConstraintMinValues(numopt_common::Vector& d);
  bool getInequalityConstraintMaxValues(numopt_common::Vector& f);

  bool getLocalInequalityConstraintJacobian(numopt_common::SparseMatrix& jacobian,
                                            const numopt_common::Parameterization& params, bool newParams = true);

 private:
  void updateNumberOfInequalityConstraints();

  Stance stance_;
  Stance supportStance_;
  LegPositions positionsBaseToHipInBaseFrame_;
  Position centerOfMassInBaseFrame_;

  size_t nSupportRegionInequalityConstraints_;
  grid_map::Polygon supportRegion_;

  size_t nLimbLengthInequalityConstraints_;
  numopt_common::Vector limbLengthInequalityConstraintsMinValues_;
  numopt_common::Vector limbLengthInequalityConstraintsMaxValues_;

  //! A from A*x <= b.
  Eigen::MatrixXd supportRegionInequalityConstraintGlobalJacobian_;
  //! b from A*x <= b.
  numopt_common::Vector supportRegionInequalityConstraintsMaxValues_;
};

} /* namespace */
