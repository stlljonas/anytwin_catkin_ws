/*
 * SplineSwingTrajectoryParameterization.hpp
 *
 *  Created on: Sep 11, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include "locomotion_planner/common/type_defs.hpp"

#include <curves/CubicHermiteE3Curve.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

#include <map>

namespace locomotion_planner {

class SplineSwingTrajectoryParameterization : public numopt_common::ParameterizationIdentity
{
 public:
  SplineSwingTrajectoryParameterization(const size_t nKnots);
  SplineSwingTrajectoryParameterization(const numopt_common::Parameterization& other);
  virtual ~SplineSwingTrajectoryParameterization();

  void setKnotPositions(std::vector<curves::CubicHermiteE3Curve::ValueType>& positions);
  std::vector<curves::CubicHermiteE3Curve::ValueType> getKnotPositions() const;

  void getTimesAndValues(const Position& startPosition, const Position& endPosition, const double averageVelocity,
                         const double minimumDuration, std::vector<curves::Time>& times,
                         std::vector<curves::CubicHermiteE3Curve::ValueType>& values);

  bool computeTrajectory(const Position& startPosition, const Position& endPosition,
                         const LinearVelocity& startVelocity, const LinearVelocity& endVelocity,
                         const double averageVelocity, const double minimumDuration);

  void getSampledValues(const double timeResolution, std::map<double, std::tuple<Position, LinearVelocity>>& values) const;
  void getSampledValues(const size_t nSamples, std::map<double, std::tuple<Position, LinearVelocity>>& values) const;

 private:
  static const size_t nDimensionsPerKnot_ = 3;
  const size_t nKnots_;
  bool isTrajectoryComputed_;
  curves::CubicHermiteE3Curve trajectory_;
};

} /* namespace free_gait */
