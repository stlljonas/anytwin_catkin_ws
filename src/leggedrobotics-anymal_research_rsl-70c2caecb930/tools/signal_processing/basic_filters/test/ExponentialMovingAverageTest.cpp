/*
 * ExponentialMovingAverageTest.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: Philipp Leemann, Aravind Vijayan
 */

// gtest
#include <gtest/gtest.h>

// basic filters
#include "basic_filters/filters.hpp"

#include <Eigen/Core>

#include <algorithm>  // std::fill(..)
#include <array>

TEST(ExponentialMovingAverage, ConstructionTest) {  // NOLINT
  basic_filters::ExponentialMovingAverageDouble filter;
  ASSERT_DOUBLE_EQ(filter.getFilteredValue(), 0.0);
  basic_filters::ExponentialMovingAverageEigenVector3d eigenFilter;
  ASSERT_EQ(eigenFilter.getFilteredValue(), Eigen::Vector3d::Zero());
}

TEST(ExponentialMovingAverage, NegativeDtTest) {  // NOLINT
  constexpr double dt = -1e-3;
  constexpr double forcedDt = 1e-3;
  constexpr double T = 0.1;
  constexpr double newVal = 100;
  basic_filters::ExponentialMovingAverageDouble filter(dt, T);
  filter.advance(newVal);
  auto expectedOutput = (1. - std::exp(-forcedDt / T)) * newVal;
  ASSERT_LE(std::abs(filter.getFilteredValue() - expectedOutput), 1e-12);
}

TEST(ExponentialMovingAverage, DoubleTest) {  // NOLINT
  std::array<double, 400> input{};
  std::array<double, 400> output{};
  std::fill(input.begin(), std::next(input.begin(), 100), 1.0);
  std::fill(std::next(input.begin(), 100), input.end(), 3.0);

  constexpr double dt = 1e-3;
  constexpr double T = 100. * dt;
  basic_filters::ExponentialMovingAverageDouble filter(dt, T, input[0]);

  for (unsigned int k = 0; k < input.size(); k++) {
    output[k] = filter.advance(input[k]);
  }

  ASSERT_DOUBLE_EQ(output[0], 1.0);
  ASSERT_DOUBLE_EQ(output[99], 1.0);
  ASSERT_TRUE(1.5 < output[199] && output[199] < 3.0);
  ASSERT_TRUE(2.5 < output[299] && output[299] < 3.0);
  ASSERT_TRUE(2.9 < output[399] && output[399] < 3.0);

  double expectedOutput199 = std::exp(-1) * 1.0 + (1. - std::exp(-1)) * 3.0;
  ASSERT_LE(std::abs(output[199] - expectedOutput199), 1e-12);
}

TEST(ExponentialMovingAverage, EigenInitTest) {  // NOLINT
  constexpr double dt = 1e-3;
  constexpr double T = 10. * dt;
  basic_filters::ExponentialMovingAverageEigenVector3d filter(dt, T);
  Eigen::Vector3d output = filter.advance(Eigen::Vector3d(0.0, 0.0, 0.0));
  ASSERT_TRUE(output == Eigen::Vector3d(0.0, 0.0, 0.0));
}

TEST(ExponentialMovingAverage, EigenTest) {  // NOLINT
  auto firstInput = Eigen::Vector3d(0.0, 1.0, 2.0);
  auto secondInput = Eigen::Vector3d(2.0, 3.0, 4.0);

  std::array<Eigen::Vector3d, 400> input;
  std::array<Eigen::Vector3d, 400> output;
  std::fill(input.begin(), std::next(input.begin(), 100), firstInput);
  std::fill(std::next(input.begin(), 100), input.end(), secondInput);

  constexpr double dt = 1e-3;
  constexpr double T = 100. * dt;
  basic_filters::ExponentialMovingAverageEigenVector3d filter(dt, T, input[0]);

  for (unsigned int k = 0; k < input.size(); k++) {
    output[k] = filter.advance(input[k]);
  }

  Eigen::Vector3d diff = secondInput - firstInput;
  Eigen::Vector3d delta = diff / diff.dot(diff);
  auto progress = [&delta, &firstInput](const Eigen::Vector3d& vec) { return (vec - firstInput).dot(delta); };

  ASSERT_DOUBLE_EQ(progress(output[0]), 0.0);
  ASSERT_DOUBLE_EQ(progress(output[99]), 0.0);
  ASSERT_GE(progress(output[199]), 0.6);
  ASSERT_GE(progress(output[299]), 0.8);
  ASSERT_GE(progress(output[399]), 0.9);

  Eigen::Vector3d expectedOutput199 = std::exp(-1) * firstInput + (1. - std::exp(-1)) * secondInput;
  ASSERT_LE((output[199] - expectedOutput199).norm(), 1e-12);
}
