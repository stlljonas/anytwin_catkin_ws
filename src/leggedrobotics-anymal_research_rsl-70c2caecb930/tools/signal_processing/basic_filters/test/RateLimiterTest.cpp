/**
 * @authors     Aravind Vijayan
 * @affiliation ANYbotics
 * @brief       Tests for rate limiting filter
 */

// gtest
#include <gtest/gtest.h>

// basic filters
#include "basic_filters/filters.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>

TEST(RateLimiter, DoubleTest) {  // NOLINT
  basic_filters::RateLimiterDouble filter1;
  filter1.initialize(0.0);
  filter1.setParameters(1.0, -1.0);
  basic_filters::RateLimiterDouble filter2(0.0);
  filter2.setParameters(1.0, -1.0);
  basic_filters::RateLimiterDouble filter3(0.0, 1.0, -1.0);

  filter1.update(1.0, 1.0);
  filter2.update(1.0, 1.0);
  filter3.update(1.0, 1.0);

  ASSERT_TRUE(filter1.getValue() == filter2.getValue());
  ASSERT_TRUE(filter2.getValue() == filter3.getValue());
}

TEST(RateLimiter, FloatTest) {  // NOLINT
  basic_filters::RateLimiterFloat filter1;
  filter1.initialize(0.0);
  filter1.setParameters(1.0, -1.0);
  basic_filters::RateLimiterFloat filter2(0.0);
  filter2.setParameters(1.0, -1.0);
  basic_filters::RateLimiterFloat filter3(0.0, 1.0, -1.0);

  filter1.update(1.0, 1.0);
  filter2.update(1.0, 1.0);
  filter3.update(1.0, 1.0);

  ASSERT_TRUE(filter1.getValue() == filter2.getValue());
  ASSERT_TRUE(filter2.getValue() == filter3.getValue());
}

TEST(RateLimiter, FiltetUpdateTest) {  // NOLINT
  basic_filters::RateLimiterDouble filter(0.0, 1.0, -1.0);
  // Inputs outside max rate of change
  filter.update(1.5, 1.0);
  ASSERT_TRUE(filter.getValue() == 1.0);
  filter.update(-1.5, 1.0);
  ASSERT_TRUE(filter.getValue() == 0.0);
  // Inputs within the rate of change
  filter.update(0.5, 1.0);
  ASSERT_TRUE(filter.getValue() == 0.5);
  filter.update(0.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 0.0);
  // Inputs at the max rate of change
  filter.update(1.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 1.0);
  filter.update(-1.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 0.0);
  filter.update(-1.0, 1.0);
  ASSERT_TRUE(filter.getValue() == -1.0);
}

TEST(RateLimiter, FilterUnitializedTest) {  // NOLINT
  basic_filters::RateLimiterDouble filter;
  filter.setParameters(1.0, -1.0);
  filter.update(1.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 1.0);
}

TEST(RateLimiter, ResetTest) {  // NOLINT
  basic_filters::RateLimiterDouble filter(0.1, 1.0, -1.0);
  filter.reset();
  ASSERT_TRUE(filter.getValue() == 0.0);
}

TEST(RateLimiter, EigenTest) {  // NOLINT
  basic_filters::RateLimiterEigenVector3d filter;
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d::Zero());

  filter.reset();
  filter.setParameters(Eigen::Vector3d::Ones(), -Eigen::Vector3d::Ones());
  filter.update(Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d::Ones());
  filter.update(-Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d::Zero());
  filter.update(-2.0 * Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == -Eigen::Vector3d::Ones());
  filter.update(2.0 * Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d::Zero());
  filter.update(0.5 * Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == 0.5 * Eigen::Vector3d::Ones());
}

TEST(RateLimiter, EigenTestUnequalValue) {  // NOLINT
  basic_filters::RateLimiterEigenVector3d filter(Eigen::Vector3d(0.0, 1.0, -1.0));
  filter.setParameters(Eigen::Vector3d::Ones(), -Eigen::Vector3d::Ones());
  filter.update(Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d(1.0, 1.0, 0.0));
  filter.update(-Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d(0.0, 0.0, -1.0));
}

TEST(RateLimiter, EigenTestUnequalRates) {  // NOLINT
  basic_filters::RateLimiterEigenVector3d filter(Eigen::Vector3d::Zero());
  filter.setParameters(Eigen::Vector3d(0.5, 1.0, 2.0), -Eigen::Vector3d(0.5, 1.0, 2.0));
  filter.update(2.0 * Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d(0.5, 1.0, 2.0));
  filter.update(-2.0 * Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d(0.0, 0.0, 0.0));
  filter.update(-2.0 * Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d(-0.5, -1.0, -2.0));
}