/**
 * @authors     Remo Diethelm
 * @affiliation ANYbotics
 * @brief       Tests for average calculation class using a test fixture.
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
 * Test Fixtures.
 */

/*!
 * Create a test fixture class.
 */
class TestAverageCalculatorUsingFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    //! Set up your test cases.

    // If you care about reproducibility of your pseudorandom numbers, then (re-)initialize the random seed here in the SetUp() method. This
    // guarantees that a specific test case produces reproducible random values during every run. Doing this in another place, e.g. main()
    // would be subject to non-determinism when tests are re-ordered, or if a failure and subsequent test case abort does not consume the
    // "usual" number of pseudorandom values.
    currentTime_ = time(nullptr);
    srand(currentTime_);
    // Record the used random seed. This is stored in the test suite's result XML file.
    RecordProperty("random_seed", currentTime_);
  }

  void TearDown() override {
    //! Clear everything with is not handled by the destructor in here.
  }

  double randValue() { return 100.0 * static_cast<double>(rand()) / RAND_MAX; }

  void addNValues(const size_t nValues) {
    // in c++20 this would probably be:
    // for (int i : std::iota_view{0, nValues})
    // c.f. https://en.cppreference.com/w/cpp/ranges/iota_view
    // But until then:
    for (int i = 0; i < nValues; i++) {
      auto value = randValue();
      averageCalculator_.addValue(value);
      sumMultipleValues_ += value;
    }
  }

 protected:
  average_calculator::AverageCalculator averageCalculator_;

  const size_t nValues_ = 100;
  double sumMultipleValues_ = 0.0;

  unsigned int currentTime_ = 0;
};

/*!
 * Create the test cases using a set up test fixture.
 */
TEST_F(TestAverageCalculatorUsingFixture, isZeroOnConstruction) {  // NOLINT
  // This test case tests the initialization case - when we're working with a freshly created AverageCalculator, we expect the average
  // to be exactly 0.0:
  ASSERT_EQ(0.0, averageCalculator_.getAverageValue());
}

TEST_F(TestAverageCalculatorUsingFixture, forSingleValue) {  // NOLINT
  // If there is exactly one value in AverageCalculator's accumulator, the average is excactly that single value:
  auto singleValue = randValue();
  averageCalculator_.addValue(singleValue);
  ASSERT_EQ(singleValue, averageCalculator_.getAverageValue());
}

TEST_F(TestAverageCalculatorUsingFixture, forMultipleValues) {  // NOLINT
  // Check the normal case: we've added a number of values, and expect the average to be the arithmetic mean. In the case of a failure,
  // we want to be able to reproduce the random seed used for value generation here. It's not strictly necessary to perform any additional
  // actions here in the test, since that value has been registered in the SetUp() method already, and is accessible through the test
  // suite's result XML. But for convenience, and to demonstrate how this can be done, we're emitting the random seed used together with a
  // potential test failure:
  addNValues(nValues_);
  auto expected = sumMultipleValues_ / nValues_;
  ASSERT_NEAR(expected, averageCalculator_.getAverageValue(), 1e-10) << "Random seed: " << currentTime_;
}

TEST_F(TestAverageCalculatorUsingFixture, isZeroAfterClearance) {  // NOLINT
  // As previously, add a number of values, then clear the AverageCalculator. The average should now again be exactly 0.0:
  addNValues(nValues_);
  averageCalculator_.clear();
  ASSERT_EQ(0.0, averageCalculator_.getAverageValue());
}
