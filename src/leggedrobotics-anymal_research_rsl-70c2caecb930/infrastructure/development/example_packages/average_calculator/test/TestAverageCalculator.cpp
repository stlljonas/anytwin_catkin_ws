/**
 * @authors     Remo Diethelm
 * @affiliation ANYbotics
 * @brief       Tests for average calculation class.
 */

#include <vector>

#include <gtest/gtest.h>

#include "average_calculator/AverageCalculator.hpp"

/*!
 * GTest offers two ways of writing tests:
 * - Simple tests:  Easy to set up, but normally leads to some code duplication.
 * - Test Fixtures: A bit more work to set up, but help to avoid code duplication and simplify maintenance.
 * In this example, the same set of tests is implemented once as simple tests, and once using test fixtures.
 *
 * GTest offers different type of assertions:
 * ASSERT -> Aborts test on failure.
 * EXPECT -> Continues test on failure.
 *
 * For more information on GTest, see https://github.com/google/googletest/blob/master/googletest/docs/primer.md.
 */

/*!
 * Simple tests.
 */

TEST(TestAverageCalculator, isZeroOnConstruction) {  // NOLINT
  average_calculator::AverageCalculator averageCalculator;
  ASSERT_EQ(0.0, averageCalculator.getAverageValue());
}

TEST(TestAverageCalculator, forSingleValue) {  // NOLINT
  const double value = 100.0 * static_cast<double>(rand()) / RAND_MAX;
  average_calculator::AverageCalculator averageCalculator;
  averageCalculator.addValue(value);
  ASSERT_EQ(value, averageCalculator.getAverageValue());
}

TEST(TestAverageCalculator, forMultipleValues) {  // NOLINT
  size_t nValues = 100;
  average_calculator::AverageCalculator averageCalculator;
  double sum = 0.0;
  for (size_t i = 0; i < nValues; i++) {
    const double value = 100.0 * static_cast<double>(rand()) / RAND_MAX;
    averageCalculator.addValue(value);
    sum += value;
  }
  ASSERT_NEAR(sum / nValues, averageCalculator.getAverageValue(), 1e-10);
}

TEST(TestAverageCalculator, isZeroAfterClearance) {  // NOLINT
  size_t nValues = 100;
  average_calculator::AverageCalculator averageCalculator;
  for (size_t i = 0; i < nValues; i++) {
    const double value = 100.0 * static_cast<double>(rand()) / RAND_MAX;
    averageCalculator.addValue(value);
  }
  averageCalculator.clear();
  ASSERT_EQ(0.0, averageCalculator.getAverageValue());
}
