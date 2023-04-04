/*
 * FirstOrderFilterTest.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: dbellicoso, Philipp Leemann
 */

// gtest
#include <gtest/gtest.h>

// basic filters
#include "basic_filters/filters.hpp"

#include <Eigen/Core>
#include <kindr/Core>

#include <algorithm>  // std::fill(..)
#include <array>

TEST(FirstOrderFilter, EigenVector3Test) {  // NOLINT
  const Eigen::Vector3d v1(0.0, 0.0, 0.0);
  const Eigen::Vector3d v2(2.0, 2.0, 2.0);

  constexpr double dt = 0.0025;
  constexpr double duration = 100.0;
  constexpr auto iterations = static_cast<unsigned int>(duration / dt);

  basic_filters::FirstOrderFilter<Eigen::Vector3d> vectorFilter(dt, 0.01, 1.0, v1);

  for (unsigned int k = 0; k < iterations; k++) {
    vectorFilter.advance(v2);
  }

  const Eigen::Vector3d filteredVector = vectorFilter.getFilteredValue();

  ASSERT_TRUE(v2.isApprox(filteredVector));
}

TEST(FirstOrderFilter, EigenVectorNTest) {  // NOLINT
  constexpr unsigned int n = 10;
  constexpr int finalVal = 2;
  using VectorNd = Eigen::Matrix<double, n, 1>;

  constexpr double dt = 0.0025;
  constexpr double duration = 200.0;
  constexpr auto iterations = static_cast<unsigned int>(duration / dt);

  // Initialize initial and final vectors.
  VectorNd finalVector;
  for (unsigned int k = 0; k < n; k++) {
    finalVector[k] = finalVal;
  }

  // Construct a filter using new operator to check for eigen alignment issues.
  // todo: this does not seem to reliably test alignment issues?
  auto* vectorFilter = new basic_filters::FirstOrderFilter<VectorNd>();
  vectorFilter->setFilterParameters(dt, 0.01, 1.0);

  for (unsigned int k = 0; k < iterations; k++) {
    vectorFilter->advance(finalVector);
  }

  const auto filteredVector = vectorFilter->getFilteredValue();

  delete vectorFilter;

  ASSERT_TRUE(finalVector.isApprox(filteredVector));
}

TEST(FirstOrderFilter, EigenMatrixXdTest) {  // NOLINT
  constexpr unsigned int n = 10;
  constexpr int finalVal = 2;

  constexpr double dt = 0.0025;
  constexpr double duration = 200.0;
  constexpr auto iterations = static_cast<unsigned int>(duration / dt);

  // Initialize initial and final vectors.
  const Eigen::MatrixXd m1 = Eigen::MatrixXd::Zero(n, n);
  Eigen::MatrixXd m2(n, n);
  for (unsigned int k = 0; k < n; k++) {
    for (unsigned int h = 0; h < n; h++) {
      m2(k, h) = finalVal;
    }
  }

  // Construct a filter with default values.
  basic_filters::FirstOrderFilter<Eigen::MatrixXd> matrixFilter(dt, 0.01, 1.0, m1);

  for (unsigned int k = 0; k < iterations; k++) {
    matrixFilter.advance(m2);
  }

  const auto filteredMatrix = matrixFilter.getFilteredValue();

  ASSERT_TRUE(m2.isApprox(filteredMatrix));
}

TEST(FirstOrderFilter, KindrPositionTest) {  // NOLINT
  constexpr int finalVal = 2;
  constexpr double dt = 0.0025;
  constexpr double duration = 200.0;
  constexpr auto iterations = static_cast<unsigned int>(duration / dt);

  // Initialize initial and final vectors.
  const kindr::Position3D p1;
  const kindr::Position3D p2(finalVal, finalVal, finalVal);

  // Construct a filter.
  basic_filters::FirstOrderFilterKindrPosition positionFilter(dt, 0.01, 1.0);

  for (unsigned int k = 0; k < iterations; k++) {
    positionFilter.advance(p2);
  }

  const auto pFiltered = positionFilter.getFilteredValue();

  ASSERT_TRUE(p2.toImplementation().isApprox(pFiltered.toImplementation()));
}

TEST(FirstOrderFilter, DoubleFilterOutputTest) {  // NOLINT
#include "test_data_first_order_filter.h"

  basic_filters::FirstOrderFilter<double> filter(dt, tau, gain);

  for (unsigned int i = 0; i < len; ++i) {
    double y_out = filter.advance(u[i]);
    //        std::cout << std::fixed << std::setprecision(15) << y_out << ", ";
    ASSERT_NEAR(y[i], y_out, 1e-4);  // have to use low precision because matlab generated data files contain doubles converted to strings
  }
}
