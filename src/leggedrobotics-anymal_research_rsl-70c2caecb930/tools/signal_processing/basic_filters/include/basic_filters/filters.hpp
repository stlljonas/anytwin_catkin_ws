/*
 * filters.hpp
 *
 *  Created on: Jan 12, 2017
 *      Author: Dario Bellicoso, Philipp Leemann
 */

#pragma once

#include "basic_filters/BrakingRateLimiter.hpp"
#include "basic_filters/ExponentialMovingAverage.hpp"
#include "basic_filters/FilterMovAvg.hpp"
#include "basic_filters/FirstOrderFilter.hpp"
#include "basic_filters/RateLimiter.hpp"

#include <Eigen/Core>
#include <kindr/Core>

namespace basic_filters {

//! Define some helper types for filters.

using ExponentialMovingAverageDouble = ExponentialMovingAverage<double>;
using ExponentialMovingAverageEigenVector3d = ExponentialMovingAverage<Eigen::Vector3d>;

using FirstOrderFilterD = FirstOrderFilter<double>;
using FirstOrderFilterF = FirstOrderFilter<float>;

using FilterMovAvgD = FilterMovAvg<double>;
using FilterMovAvgF = FilterMovAvg<float>;

using RateLimiterDouble = RateLimiter<double>;
using RateLimiterFloat = RateLimiter<float>;
using RateLimiterEigenVector3d = RateLimiter<Eigen::Vector3d>;

using BrakingRateLimiterDouble = BrakingRateLimiter<double>;
using BrakingRateLimiterFloat = BrakingRateLimiter<float>;
using BrakingRateLimiterEigenVector3d = BrakingRateLimiter<Eigen::Vector3d>;

using FirstOrderFilterEigenMatrix3d = FirstOrderFilter<Eigen::Matrix3d>;
using FirstOrderFilterEigenVector3d = FirstOrderFilter<Eigen::Vector3d>;

using FirstOrderFilterKindrVector3d = FirstOrderFilter<kindr::VectorTypeless3D>;
using FirstOrderFilterKindrPosition = FirstOrderFilter<kindr::Position3D>;
using FirstOrderFilterKindrLinearVelocity = FirstOrderFilter<kindr::Velocity3D>;
using FirstOrderFilterKindrAngularVelocity = FirstOrderFilter<kindr::LocalAngularVelocityD>;

template <typename ValueType, unsigned int Rows, unsigned int Cols>
using FirstOrderFilterEigenMatrixFixed = FirstOrderFilter<Eigen::Matrix<ValueType, Rows, Cols>>;

}  // namespace basic_filters
