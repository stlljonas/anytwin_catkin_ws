/*
 * SplineSwingTrajectoryParameterization.cpp
 *
 *  Created on: Sep 11, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "locomotion_planner/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryParameterization.hpp"

#include <free_gait_core/leg_motion/Footstep.hpp>

namespace locomotion_planner {

SplineSwingTrajectoryParameterization::SplineSwingTrajectoryParameterization(const size_t nKnots)
    : nKnots_(nKnots),
      isTrajectoryComputed_(false)
{
  params_.resize(nKnots_ * nDimensionsPerKnot_);
}

SplineSwingTrajectoryParameterization::SplineSwingTrajectoryParameterization(
    const numopt_common::Parameterization& other)
    : isTrajectoryComputed_(false),
      nKnots_(other.getGlobalSize() / nDimensionsPerKnot_)
{
  params_ = other.getParams();
}

SplineSwingTrajectoryParameterization::~SplineSwingTrajectoryParameterization()
{
}

void SplineSwingTrajectoryParameterization::setKnotPositions(std::vector<curves::CubicHermiteE3Curve::ValueType>& positions)
{
  if (positions.size() != nKnots_) {
    throw std::range_error("SplineSwingTrajectoryParameterization: Number of knot positions does not comply with the number of knots!");
  }

  for (size_t i = 0; i < nKnots_; ++i) {
     params_.segment(i * nDimensionsPerKnot_, nDimensionsPerKnot_) = positions[i];
  }
}

std::vector<curves::CubicHermiteE3Curve::ValueType> SplineSwingTrajectoryParameterization::getKnotPositions() const
{
  std::vector<curves::CubicHermiteE3Curve::ValueType> knotPositions;
  knotPositions.reserve(nKnots_);
  for (size_t i = 0; i < nKnots_; ++i) {
    knotPositions.push_back(params_.segment(i * nDimensionsPerKnot_, nDimensionsPerKnot_));
  }
  return knotPositions;
}

void SplineSwingTrajectoryParameterization::getTimesAndValues(const Position& startPosition,
                                                              const Position& endPosition,
                                                              const double averageVelocity,
                                                              const double minimumDuration,
                                                              std::vector<curves::Time>& times,
                                                              std::vector<curves::CubicHermiteE3Curve::ValueType>& values)
{
  values.clear();
  values.reserve(nKnots_ + 2);
  values.push_back(startPosition.vector());
  for (size_t i = 0; i < nKnots_; ++i) {
    values.push_back(params_.segment(i * nDimensionsPerKnot_, nDimensionsPerKnot_));
  }
  values.push_back(endPosition.vector());

  times.clear();
  free_gait::Footstep::computeTiming(values, averageVelocity, minimumDuration, times);
}

bool SplineSwingTrajectoryParameterization::computeTrajectory(const Position& startPosition,
                                                              const Position& endPosition,
                                                              const LinearVelocity& startVelocity,
                                                              const LinearVelocity& endVelocity,
                                                              const double averageVelocity,
                                                              const double minimumDuration)
{
  std::vector<curves::Time> times;
  std::vector<curves::CubicHermiteE3Curve::ValueType> values;
  getTimesAndValues(startPosition, endPosition, averageVelocity, minimumDuration, times, values);
  trajectory_.fitCurveWithDerivatives(times, values, startVelocity.vector(), endVelocity.vector());
  isTrajectoryComputed_ = true;
  return true;
}

void SplineSwingTrajectoryParameterization::getSampledValues(const double timeResolution, std::map<double, std::tuple<Position, LinearVelocity>>& values) const
{
  if (!isTrajectoryComputed_) throw std::runtime_error("SplineSwingTrajectoryParameterization: Trajectory is not yet computed!");
  values.clear();
  const double duration = trajectory_.getMaxTime() - trajectory_.getMinTime();
  const size_t nSamples = std::ceil(duration / timeResolution);
  return getSampledValues(nSamples, values);
}

void SplineSwingTrajectoryParameterization::getSampledValues(const size_t nSamples, std::map<double, std::tuple<Position, LinearVelocity>>& values) const
{
  if (!isTrajectoryComputed_) throw std::runtime_error("SplineSwingTrajectoryParameterization: Trajectory is not yet computed!");
  values.clear();
  const double duration = trajectory_.getMaxTime() - trajectory_.getMinTime();
  const double dt = duration / nSamples;
  for (size_t k = 0; k < nSamples; ++k) {
    const double time = (double)k * dt;
    Position position;
    trajectory_.evaluate(position.vector(), time);
    LinearVelocity velocity;
    trajectory_.evaluateDerivative(velocity.vector(), time, 1);
    values[time] = std::tuple<Position, LinearVelocity>(position, velocity);
  }
}

} /* namespace */
