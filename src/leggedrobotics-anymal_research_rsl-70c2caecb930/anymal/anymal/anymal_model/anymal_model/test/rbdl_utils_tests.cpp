/*!
 * @file    rbdl_utils_tests.cpp
 * @author  Christian Gehring, Dario Bellicoso
 * @date    Oct, 2015
 */

#include <gtest/gtest.h>
#include <anymal_model/rbdl_utils.hpp>
#include "TestAnymalModel.hpp"
#include "gtest_anymal_model.hpp"
#include "kindr/common/gtest_eigen.hpp"

class RbdlUtilsTest : public TestAnymalModel {
 public:
};

TEST_F(RbdlUtilsTest, setRbdlQFromState) {
  initModel("starleth_unit_test");

  Eigen::VectorXd rbdlQ;
  AnymalState state;
  state.setPositionWorldToBaseInWorldFrame(Position(0.1, 0.2, 0.3));
  RotationQuaternion orientationBaseToWorld(EulerAnglesZyx(0.1, 0.2, 0.3));
  state.setOrientationBaseToWorld(orientationBaseToWorld);

  setRbdlQFromState(rbdlQ, state);

  EXPECT_EQ(0.1, rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::X)));
  EXPECT_EQ(0.2, rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Y)));
  EXPECT_EQ(0.3, rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Z)));
  EXPECT_EQ(orientationBaseToWorld.w(), rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_W)));
  EXPECT_EQ(orientationBaseToWorld.x(), rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_X)));
  EXPECT_EQ(orientationBaseToWorld.y(), rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_Y)));
  EXPECT_EQ(orientationBaseToWorld.z(), rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_Z)));
}

TEST_F(RbdlUtilsTest, convertRbdlQToAnymalState) {
  initModel("starleth_unit_test", USE_QUATERNION);

  AnymalState stateIn;
  AnymalState stateOut;
  Eigen::VectorXd rbdlQ;
  stateIn.setRandom();

  setRbdlQFromState(rbdlQ, stateIn);
  setStateFromRbdlQ(stateOut, rbdlQ);

  for (int i = 0; i < stateIn.getJointPositions().Dimension; i++) {
    EXPECT_EQ(stateIn.getJointPositions()(i), stateOut.getJointPositions()(i));
  }
  EXPECT_EQ(stateIn.getPositionWorldToBaseInWorldFrame().x(), stateOut.getPositionWorldToBaseInWorldFrame().x());
  EXPECT_EQ(stateIn.getPositionWorldToBaseInWorldFrame().y(), stateOut.getPositionWorldToBaseInWorldFrame().y());
  EXPECT_EQ(stateIn.getPositionWorldToBaseInWorldFrame().z(), stateOut.getPositionWorldToBaseInWorldFrame().z());
  ASSERT_TRUE(stateIn.getOrientationBaseToWorld() == stateOut.getOrientationBaseToWorld());
}
