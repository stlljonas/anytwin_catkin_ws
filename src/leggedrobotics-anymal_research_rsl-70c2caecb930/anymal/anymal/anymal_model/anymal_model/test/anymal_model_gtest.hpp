/*!
 * @file    anymal_model_gtest.hpp
 * @author  Christian Gehring
 * @date    Oct, 2015
 */

#pragma once

#include <gtest/gtest.h>

#include <kindr/common/gtest_eigen.hpp>

namespace anymal_model {

void expectNear(AnymalState& expectedState, AnymalState& state, double tol = 1e-2) {
  KINDR_ASSERT_DOUBLE_MX_EQ(expectedState.getPositionWorldToBaseInWorldFrame().toImplementation(),
                            state.getPositionWorldToBaseInWorldFrame().toImplementation(), tol, "positionWorldToBaseInWorldFrame");
  KINDR_ASSERT_DOUBLE_MX_EQ(expectedState.getJointPositions().toImplementation(), state.getJointPositions().toImplementation(), tol,
                            "Joint positions");
  EXPECT_NEAR(0.0, expectedState.getOrientationBaseToWorld().getDisparityAngle(state.getOrientationBaseToWorld()), tol);
}

}  // namespace anymal_model
