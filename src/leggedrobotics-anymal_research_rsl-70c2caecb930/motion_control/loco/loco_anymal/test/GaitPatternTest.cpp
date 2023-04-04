/**
 * @authors     Dario Bellicoso
 * @affiliation RSL
 * @brief       Gait pattern tests.
 */

// gtest
#include <gtest/gtest.h>

// loco
#include <loco/gait_pattern/GaitPatternFlightPhases.hpp>

// loco anymal
#include "loco_anymal/testing/CommonFixture.hpp"

namespace loco_anymal {

class GaitPatternTests : public CommonFixture {};

TEST_F(GaitPatternTests, testCrawlingPattern) {
  // Setup gait pattern.
  const auto strideDuration = 5.0;

  auto gaitPattern = loco::GaitPatternFlightPhases(*wholeBody_);
  gaitPattern.clear();
  gaitPattern.setStrideDuration(strideDuration);
  gaitPattern.addFootFallPattern(3, 0.00, 0.20);
  gaitPattern.addFootFallPattern(1, 0.25, 0.45);
  gaitPattern.addFootFallPattern(2, 0.50, 0.70);
  gaitPattern.addFootFallPattern(0, 0.75, 0.95);

  // Test.
  ASSERT_NEAR(strideDuration, gaitPattern.getStrideDuration(), 1e-8);
  ASSERT_NEAR(0.20 * strideDuration, gaitPattern.getSwingDuration(0), 1e-8);
  ASSERT_NEAR(0.80 * strideDuration, gaitPattern.getStanceDuration(0), 1e-8);

  // Test swing phases.
  ASSERT_NEAR(0.5, gaitPattern.getSwingPhaseForLeg(3, 0.10), 1e-8);
  ASSERT_NEAR(1.0, gaitPattern.getSwingPhaseForLeg(3, 0.20), 1e-8);
  ASSERT_NEAR(-1.0, gaitPattern.getSwingPhaseForLeg(3, 0.30), 1e-8);

  gaitPattern.lock(false);
  gaitPattern.advance(dt_);
  ASSERT_NEAR(dt_, gaitPattern.getSwingPhaseForLeg(3), 1e-8);
}

} // namespace loco_anymal
