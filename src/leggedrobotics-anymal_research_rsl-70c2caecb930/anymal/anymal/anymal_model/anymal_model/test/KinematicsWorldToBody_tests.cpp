/*!
 * @file    KinematicsWorldToBody_tests.cpp
 * @author  Dario Bellicoso
 * @date    Sep, 2015
 */

#include <gtest/gtest.h>
#include <iostream>
#include <kindr/Core>
#include "TestAnymalModel.hpp"
#include "gtest_anymal_model.hpp"
#include "kindr/common/gtest_eigen.hpp"

using AnymalModel = anymal_model::AnymalModel;

class KinematicsWorldToBodyTest : public TestAnymalModel {
 public:
};

TEST_F(KinematicsWorldToBodyTest, testRotationMatrixOrthonormality) {
  initModel("starleth_unit_test", USE_QUATERNION);

  AnymalState state;
  state.setRandom();
  getModelPtr()->setState(state, true);

  for (auto bodyKey : internal::bodyKeys) {
    const BodyEnum bodyEnum = bodyKey.first;
    const std::string msg = "testRotationMatrixDefinition for body named " + getModelPtr()->getBodyContainer()[bodyEnum].getName();
    const Eigen::Matrix3d& rotationMatrix = getModelPtr()->getOrientationWorldToBody(bodyEnum);
    const double determinant = rotationMatrix.determinant();

    EXPECT_NEAR(1.0, determinant, 1e-10);
    Eigen::MatrixXd identiy = rotationMatrix.transpose() * rotationMatrix;
    KINDR_ASSERT_DOUBLE_MX_EQ(identiy, Eigen::Matrix3d::Identity(), 1.0, msg);
  }
}

TEST_F(KinematicsWorldToBodyTest, stateAndModel) {
  initModel("starleth_unit_test", USE_QUATERNION);
  AnymalState state;
  state.setRandom();
  getModelPtr()->setState(state, true);

  RotationMatrix orientationWorldToBaseFromModelKindr = RotationMatrix(getModelPtr()->getOrientationWorldToBody(BodyEnum::BASE));
  RotationMatrix orientationWorldToBaseFromStateKindr = RotationMatrix(getModelPtr()->getState().getOrientationBaseToWorld().inverted());

  ASSERT_TRUE(getModelPtr()->getState().getOrientationBaseToWorld().inverted().isNear(orientationWorldToBaseFromModelKindr, 1.0e-3))
      << "quaternion check";
  EXPECT_NEAR(0.0, orientationWorldToBaseFromModelKindr.getDisparityAngle(orientationWorldToBaseFromStateKindr), 1.0e-3);
}

TEST_F(KinematicsWorldToBodyTest, rotatePosition) {
  // rotating a position vector by the state orientation and the model should yield the same result.

  initModel("starleth_unit_test", USE_QUATERNION);
  AnymalState state;
  state.setRandom();
  getModelPtr()->setState(state, true);

  Eigen::Vector3d positionA(-1.2, 3.4, 0.9);

  const auto orientationA = state.getOrientationBaseToWorld().inverted();
  const auto orientationAB = RotationQuaternion(EulerAnglesZyx(0.5, -0.8, 1.4));
  const auto orientationB = orientationAB * orientationA;
  state.setOrientationBaseToWorld(orientationB.inverted());
  getModelPtr()->setState(state, true);
  Eigen::Vector3d modelPositionB = getModelPtr()->getOrientationWorldToBody(BodyEnum::BASE) * positionA;
  Eigen::Vector3d statePositionB = orientationB.rotate(positionA);

  EXPECT_NEAR(statePositionB.x(), modelPositionB.x(), 1.0e-3);
  EXPECT_NEAR(statePositionB.y(), modelPositionB.y(), 1.0e-3);
  EXPECT_NEAR(statePositionB.z(), modelPositionB.z(), 1.0e-3);
}
