/*!
 * @file    AnymalModel_test.cpp
 * @author  Christian Gehring, Dario Bellicoso
 * @date    Oct, 2015
 */

// test
#include <gtest/gtest.h>
#include "TestAnymalModel.hpp"

// kindr
#include <kindr/Core>
#include "kindr/common/gtest_eigen.hpp"

// kinematics
#include "starleth_kinematics/jacobiansLeftForeStarleth.hpp"

// model
#include "anymal_model/AnymalModel.hpp"

using anymal_model::AD;

class KinematicsWorldToBaseTest : public TestAnymalModel {};

TEST_F(KinematicsWorldToBaseTest, getOrientationWorldToBody) {  // NOLINT
  initModel("starleth_unit_test");

  anymal_model::AnymalState state;
  state.setRandom();
  getModelPtr()->setState(state, true, false, false);
  anymal_model::RotationQuaternion orientation =
      anymal_model::RotationQuaternion(anymal_model::RotationMatrix(getModelPtr()->getOrientationWorldToBody(AD::BodyEnum::BASE)));
  EXPECT_NEAR(0.0, state.getOrientationBaseToWorld().inverted().getDisparityAngle(orientation), 1e-2);
}

TEST_F(KinematicsWorldToBaseTest, getRotationalAndRotationalDerivativeJacobianWorldToBaseInWorldFrame) {  // NOLINT
  initModel("starleth_unit_test");

  anymal_model::AnymalState state;
  state.setRandom();
  getModelPtr()->setState(state, true, false, false);

  // store some of the kinematics
  const anymal_model::GeneralizedCoordinates& generalizedPositionsQuaternion = state.getGeneralizedCoordinates();
  const anymal_model::GeneralizedVelocities& generalizedVelocities = state.getGeneralizedVelocities();

  const anymal_model::EulerAnglesXyz eulerAnglesXyz = anymal_model::EulerAnglesXyz(state.getOrientationBaseToWorld()).getUnique();
  const anymal_model::LocalAngularVelocity angularVelocityBaseInBaseFrame(generalizedVelocities.segment<3>(3));
  // const anymal_model::LocalAngularVelocity angularVelocityBaseInWorldFrame =
  // rotationWorldToBase.inverseRotate(angularVelocityBaseInBaseFrame);
  anymal_model::EulerAnglesXyzDiff eulerAnglesXyzDiff(eulerAnglesXyz.getMappingFromLocalAngularVelocityToDiff() *
                                                      angularVelocityBaseInBaseFrame.vector());

  // get the main body attitude using euler xyz angles
  Eigen::VectorXd generalizedPositionsEulerXyz(18);
  generalizedPositionsEulerXyz.setZero();
  generalizedPositionsEulerXyz.block<3, 1>(0, 0) = generalizedPositionsQuaternion.segment<3>(0);
  generalizedPositionsEulerXyz.block<3, 1>(3, 0) = eulerAnglesXyz.toImplementation();
  generalizedPositionsEulerXyz.block<12, 1>(6, 0) = state.getJointPositions().toImplementation();
  Eigen::VectorXd generalizedVelocitiesEulerAnglesXyz = generalizedVelocities;
  generalizedVelocitiesEulerAnglesXyz.block<3, 1>(3, 0) = eulerAnglesXyzDiff.toImplementation();

  // get jacobian time derivative from anymal model (spatial jacobian method)
  Eigen::MatrixXd jacobianSpatialDerivativeWorldToBaseInWorldFrame = Eigen::MatrixXd::Zero(6, 18);
  getModelPtr()->getJacobianSpatialTimeDerivativeWorldToBody(jacobianSpatialDerivativeWorldToBaseInWorldFrame, AD::BranchEnum::BASE,
                                                             AD::BodyNodeEnum::BASE, AD::CoordinateFrameEnum::WORLD);
  Eigen::MatrixXd jacobianRotationDerivativeWorldToBaseInWorldFrameFromAnymalModel =
      jacobianSpatialDerivativeWorldToBaseInWorldFrame.block<3, 18>(0, 0);

  // get conversions between euler angles xyz and angular velocity in base frame
  const Eigen::Matrix3d transformOmegaInBaseFrameToEulerAnglesXyzDiff = eulerAnglesXyz.getMappingFromLocalAngularVelocityToDiff();
  Eigen::Matrix3d transformDerivativeOmegaInBaseFrameToEulerAnglesXyzDiff =
      anymal_model::EulerAnglesXyzDiff(eulerAnglesXyzDiff.vector()).getMappingFromLocalAngularVelocityToSecondDiff(eulerAnglesXyz);

  // get jacobians from robot model
  Eigen::MatrixXd jacobianRotationWorldToBaseInWorldFrameFromRobotModel =
      starleth_kinematics::getJacobianRotationWorldToBaseInWorldFrame(generalizedPositionsEulerXyz);
  Eigen::MatrixXd jacobianRotationDerivativeWorldToBaseInWorldFrameFromRobotModel =
      starleth_kinematics::getJacobianRotationDerivativeWorldToBaseInWorldFrame(generalizedPositionsEulerXyz,
                                                                                generalizedVelocitiesEulerAnglesXyz);

  // convert main body orientation from euler angles xyz diff to angular velocity
  Eigen::MatrixXd jacobianDerivativeWorldToBaseInWorldFrameFromRobotModelConverted =
      jacobianRotationDerivativeWorldToBaseInWorldFrameFromRobotModel;
  const Eigen::Matrix3d jacobianFromRobotModel_A = jacobianRotationWorldToBaseInWorldFrameFromRobotModel.block<3, 3>(0, 3);
  const Eigen::Matrix3d jacobianDerivativeFromRobotModel_A =
      jacobianRotationDerivativeWorldToBaseInWorldFrameFromRobotModel.block<3, 3>(0, 3);
  jacobianDerivativeWorldToBaseInWorldFrameFromRobotModelConverted.block<3, 3>(0, 3) =
      jacobianFromRobotModel_A * transformDerivativeOmegaInBaseFrameToEulerAnglesXyzDiff +
      jacobianDerivativeFromRobotModel_A * transformOmegaInBaseFrameToEulerAnglesXyzDiff;

  std::string msg;
  KINDR_ASSERT_DOUBLE_MX_EQ(jacobianDerivativeWorldToBaseInWorldFrameFromRobotModelConverted,
                            jacobianRotationDerivativeWorldToBaseInWorldFrameFromAnymalModel, 1.0e-10, msg);
}

TEST_F(KinematicsWorldToBaseTest, MainBodyRotation) {  // NOLINT
  initModel("starleth_unit_test");

  anymal_model::AnymalState state;
  state.setRandom();
  getModelPtr()->setState(state, true, false, false);

  const Eigen::MatrixXd orientationWorldToBaseFromModel = getModelPtr()->getOrientationWorldToBody(AD::BodyEnum::BASE);
  const Eigen::MatrixXd orientationWorldToBaseFromState =
      anymal_model::RotationMatrix(state.getOrientationBaseToWorld().inverted()).matrix();
  const Eigen::MatrixXd orientationWorldToBaseFromModelState =
      anymal_model::RotationMatrix(getModelPtr()->getState().getOrientationBaseToWorld().inverted()).matrix();

  std::string msg;

  KINDR_ASSERT_DOUBLE_MX_EQ(orientationWorldToBaseFromModel, orientationWorldToBaseFromState, 1.0e-10, msg);
  KINDR_ASSERT_DOUBLE_MX_EQ(orientationWorldToBaseFromModel, orientationWorldToBaseFromModelState, 1.0e-10, msg);
}
