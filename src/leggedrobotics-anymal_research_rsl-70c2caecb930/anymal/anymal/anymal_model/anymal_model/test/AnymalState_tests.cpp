/*!
 * @file    AnymalState_tests.cpp
 * @author  Christian Gehring, Dario Bellicoso
 * @date    Oct, 2015
 */

#include <gtest/gtest.h>
#include "TestAnymalModel.hpp"
#include "anymal_model_gtest.hpp"
#include "kindr/common/gtest_eigen.hpp"

using anymal_model::AD;

TEST(AnymalStateTest, setGeneralizedCoordinatesToLinearlyInterpolated) {  // NOLINT
  anymal_model::AnymalState state0;
  state0.setPositionWorldToBaseInWorldFrame(anymal_model::Position(1.0, 2.0, 3.0));
  for (unsigned int i = 0; i < AD::getJointsDimension(); i++) {
    state0.getJointPositions()(i) = i;
  }
  state0.setOrientationBaseToWorld(anymal_model::RotationQuaternion(anymal_model::EulerAnglesZyx(0.0, 0.0, 0.0)));

  anymal_model::AnymalState state1;
  state1.setPositionWorldToBaseInWorldFrame(anymal_model::Position(3.0, 4.0, 5.0));
  for (unsigned int i = 0; i < AD::getJointsDimension(); i++) {
    state1.getJointPositions()(i) = i + 2;
  }
  state1.setOrientationBaseToWorld(anymal_model::RotationQuaternion(anymal_model::EulerAnglesZyx(0.0, 0.0, M_PI).inverted()));

  anymal_model::AnymalState state;
  state.setGeneralizedCoordinatesToLinearlyInterpolated(0.5, state0, state1);

  anymal_model::AnymalState expectedState;
  expectedState.setPositionWorldToBaseInWorldFrame(anymal_model::Position(2.0, 3.0, 4.0));
  for (unsigned int i = 0; i < AD::getJointsDimension(); i++) {
    expectedState.getJointPositions()(i) = (static_cast<double>(i + i + 2)) / 2.0;
  }
  expectedState.setOrientationBaseToWorld(anymal_model::RotationQuaternion(anymal_model::EulerAnglesZyx(0.0, 0.0, M_PI / 2.0).inverted()));

  state.setGeneralizedCoordinatesToLinearlyInterpolated(0.0, state0, state1);
  expectNear(state0, state, 1e-2);

  state.setGeneralizedCoordinatesToLinearlyInterpolated(0.0, state1, state1);
  expectNear(state1, state, 1e-2);
}
