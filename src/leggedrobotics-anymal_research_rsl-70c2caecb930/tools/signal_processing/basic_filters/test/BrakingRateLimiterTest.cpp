/**
 * @authors     Aravind Vijayan
 * @affiliation ANYbotics
 * @brief       Tests for braking rate limiter filter
 */

// gtest
#include <gtest/gtest.h>

// basic filters
#include "basic_filters/filters.hpp"

#include <Eigen/Core>

#include <algorithm>

TEST(BrakingRateLimiter, DoubleTest) {  // NOLINT
  basic_filters::BrakingRateLimiterDouble filter1;
  filter1.initialize(0.0);
  filter1.setParameters(1.0, 1.0);
  basic_filters::BrakingRateLimiterDouble filter2(0.0);
  filter2.setParameters(1.0, 1.0);
  basic_filters::BrakingRateLimiterDouble filter3(0.0, 1.0, 1.0);

  filter1.update(1.0, 1.0);
  filter2.update(1.0, 1.0);
  filter3.update(1.0, 1.0);

  ASSERT_TRUE(filter1.getValue() == filter2.getValue());
  ASSERT_TRUE(filter2.getValue() == filter3.getValue());
}

TEST(BrakingRateLimiter, FilterUpdateTest) {  // NOLINT
  basic_filters::BrakingRateLimiterDouble filter(0.0, 1.0, 1.0);
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
  filter.update(0.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 0.0);
  filter.update(-1.0, 1.0);
  ASSERT_TRUE(filter.getValue() == -1.0);
}

TEST(BrakingRateLimiter, FilterUnitializedTest) {  // NOLINT
  basic_filters::BrakingRateLimiterDouble filter;
  filter.setParameters(1.0, 1.0);
  filter.update(1.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 1.0);
}

TEST(BrakingRateLimiter, ResetTest) {  // NOLINT
  basic_filters::BrakingRateLimiterDouble filter(0.1, 1.0, 1.0);
  filter.reset();
  ASSERT_TRUE(filter.getValue() == 0.0);
}

TEST(BrakingRateLimiter, EigenTest) {  // NOLINT
  basic_filters::BrakingRateLimiterEigenVector3d filter;
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d::Zero());

  filter.reset();
  filter.setParameters(Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
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

TEST(BrakingRateLimiter, EigenTestUnequalValue) {  // NOLINT
  basic_filters::BrakingRateLimiterEigenVector3d filter(Eigen::Vector3d(0.0, 1.0, -1.0));
  filter.setParameters(Eigen::Vector3d::Ones(), 0.5 * Eigen::Vector3d::Ones());
  filter.update(Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d(1.0, 1.0, -0.5));
  filter.update(-Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d(0.5, 0.5, -1.0));
}

TEST(BrakingRateLimiter, EigenTestUnequalRates) {  // NOLINT
  basic_filters::BrakingRateLimiterEigenVector3d filter(Eigen::Vector3d::Zero());
  filter.setParameters(Eigen::Vector3d(0.5, 5.0, 2.0), Eigen::Vector3d(2.0, 1.0, 0.5));
  filter.update(10.0 * Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d(0.5, 5.0, 2.0));
  filter.update(-10.0 * Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d(-1.5, 4.0, 1.5));
  filter.update(-10.0 * Eigen::Vector3d::Ones(), 1.0);
  ASSERT_TRUE(filter.getValue() == Eigen::Vector3d(-2.0, 3.0, 1.0));
}

TEST(BrakingRateLimiter, AllRateCases) {  // NOLINT
  // Eight different cases to be tested.
  // Let input = i, current value = v
  // Case 1 : +ve i and +ve v, when (i = v), o/p = same
  basic_filters::BrakingRateLimiterDouble filter(5.0);
  filter.setParameters(0.1, 5.0);
  filter.update(5.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 5.0);

  // Case 2 : +ve i and +ve v, when (i > v), o/p = rate limited
  filter.initialize(5.0);
  filter.setParameters(0.1, 5.0);
  filter.update(7.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 5.1);

  // Case 3 : +ve i and +ve v, when (i < v), o/p = brakes
  filter.initialize(7.0);
  filter.setParameters(0.1, 5.0);
  filter.update(5.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 5.0);

  // Case 4 : -ve i and -ve v, when (i = v), o/p = same
  filter.initialize(-5.0);
  filter.setParameters(0.1, 5.0);
  filter.update(-5.0, 1.0);
  ASSERT_TRUE(filter.getValue() == -5.0);

  // Case 5 : -ve i and -ve v, when (i > v), o/p = rate limited
  filter.initialize(-5.0);
  filter.setParameters(0.1, 5.0);
  filter.update(-7.0, 1.0);
  ASSERT_TRUE(filter.getValue() == -5.1);

  // Case 6 : -ve i and -ve v, when (i < v), o/p = brakes
  filter.initialize(-7.0);
  filter.setParameters(0.1, 5.0);
  filter.update(-5.0, 1.0);
  ASSERT_TRUE(filter.getValue() == -5.0);

  // Case 7 : +ve i and -ve v, o/p = brakes
  filter.initialize(-5.0);
  filter.setParameters(0.1, 5.0);
  filter.update(5.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 0.0);

  // Case 8 : -ve i and +ve v, o/p = brakes
  filter.initialize(5.0);
  filter.setParameters(0.1, 5.0);
  filter.update(-5.0, 1.0);
  ASSERT_TRUE(filter.getValue() == 0.0);
}

TEST(BrakingRateLimiter, UpdateRateTest) {  // NOLINT
  basic_filters::BrakingRateLimiterDouble filter(5.0);
  filter.setParameters(1.0, 5.0);
  filter.update(10.0, 1.0);
  ASSERT_DOUBLE_EQ(filter.getValue(), 6.0);

  filter.update(10.0, 0.1);
  ASSERT_DOUBLE_EQ(filter.getValue(), 6.1);

  filter.update(-10.0, 1.0);
  ASSERT_DOUBLE_EQ(filter.getValue(), 1.1);

  filter.update(-10.0, 0.1);
  ASSERT_DOUBLE_EQ(filter.getValue(), 0.6);
}