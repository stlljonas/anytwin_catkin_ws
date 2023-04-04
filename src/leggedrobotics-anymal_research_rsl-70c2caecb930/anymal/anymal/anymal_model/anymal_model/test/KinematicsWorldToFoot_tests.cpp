/*!
 * @file    KinematicsWorldToFoot_tests.cpp
 * @author  Christian Gehring, Dario Bellicoso
 * @date    Sep, 2015
 */

#include <gtest/gtest.h>
#include <chrono>
#include <kindr/Core>
#include "TestAnymalModel.hpp"
#include "gtest_anymal_model.hpp"
#include "kindr/common/gtest_eigen.hpp"
#include "starleth_kinematics/jacobiansLeftForeStarleth.hpp"

using namespace anymal_model;

class KinematicsWorldToFootTest : public TestAnymalModel {
 public:
};

TEST_F(KinematicsWorldToFootTest, getTranslationalJacobianWorldToFootInWorldFrameFromProneuAndMathematicalConsistency) {
  initModel("starleth_unit_test", USE_QUATERNION);

  int legId = 0;
  BranchEnum branch = getModelPtr()->getBranchEnumFromLimbUInt(legId);

  AnymalState state;
  state.setRandom();
  //  state.setOrientationWorldToBase(RotationQuaternion(EulerAnglesXyz(0.0, 0.0, M_PI_2)).inverted());
  getModelPtr()->setState(state, true);

  const RotationMatrix rotationWorldToBase = RotationMatrix(getModelPtr()->getOrientationWorldToBody(BodyEnum::BASE));
  //  EulerAnglesXyz eulerAnglesXyz = EulerAnglesXyz(rotationWorldToBase);
  EulerAnglesXyz eulerAnglesXyz(state.getOrientationBaseToWorld());
  // get state representing attitude with euler xyz
  Eigen::VectorXd generalizedPositionsEulerAnglesXyz(18);
  generalizedPositionsEulerAnglesXyz.setZero();
  generalizedPositionsEulerAnglesXyz.block<3, 1>(0, 0) = state.getPositionWorldToBaseInWorldFrame().toImplementation();
  generalizedPositionsEulerAnglesXyz.block<3, 1>(3, 0) = eulerAnglesXyz.vector();
  generalizedPositionsEulerAnglesXyz.block<12, 1>(6, 0) = state.getJointPositions().toImplementation();

  // get jacobian from starleth robot model (proneu)
  Eigen::MatrixXd jacobianWorldToLFFootInWorldFrameFromRobotModel =
      getJacobianWorldToLFFootInWorldFrameFromRobotModel(generalizedPositionsEulerAnglesXyz);

  // convert representation to angular velocity

  if (USE_QUATERNION) {
    jacobianWorldToLFFootInWorldFrameFromRobotModel.block<3, 3>(0, 3) =
        jacobianWorldToLFFootInWorldFrameFromRobotModel.block<3, 3>(0, 3) * eulerAnglesXyz.getMappingFromLocalAngularVelocityToDiff();
  } else {
    jacobianWorldToLFFootInWorldFrameFromRobotModel.block<3, 3>(0, 3) =
        jacobianWorldToLFFootInWorldFrameFromRobotModel.block<3, 3>(0, 3) * eulerAnglesXyz.getMappingFromLocalAngularVelocityToDiff() *
        kindr::EulerAnglesZyxD(state.getOrientationBaseToWorld()).getMappingFromDiffToLocalAngularVelocity();
  }

  // jacobian from anymal model
  Eigen::MatrixXd jacobianWorldToLFFootInWorldFrameFromAnymalModel(6, 18);
  jacobianWorldToLFFootInWorldFrameFromAnymalModel.setZero();
  getModelPtr()->getJacobianSpatialWorldToBody(jacobianWorldToLFFootInWorldFrameFromAnymalModel, branch, BodyNodeEnum::FOOT,
                                               CoordinateFrameEnum::WORLD);

  Eigen::MatrixXd jacobianTranslationWorldToLFFootInWorldFrame = jacobianWorldToLFFootInWorldFrameFromAnymalModel.block<3, 18>(3, 0);
  Eigen::Matrix3d Jt_A = jacobianTranslationWorldToLFFootInWorldFrame.block<3, 3>(0, 3);

  std::string msg = "";
  KINDR_ASSERT_DOUBLE_MX_EQ_ZT(jacobianWorldToLFFootInWorldFrameFromRobotModel, jacobianTranslationWorldToLFFootInWorldFrame, 1.0e-3, msg,
                               1.0e-10);

  // assert that J(1:3,4:6) = -R_IB * S(B_r_B,LF)
  const Eigen::Vector3d B_r_B_LF =
      getModelPtr()->getPositionBodyToBody(BodyEnum::BASE, branch, BodyNodeEnum::FOOT, CoordinateFrameEnum::BASE);

  /*
   * The the contribution of the angular velocity of the base to the linear velocity of the limb endpoint is given by:
   *  I_v_I_F = I_w_B x I_r_B_LF
   *          = -(I_r_B_LF x I_w_B)
   *          = -S(I_r_B_LF)*I_w_B
   *          = -C_IB*S(B_r_B_LF)*B_w_B
   */
  Eigen::Matrix3d A = -rotationWorldToBase.inverseRotate(kindr::getSkewMatrixFromVector(B_r_B_LF));
  if (!USE_QUATERNION) {
    A *= kindr::EulerAnglesZyxD(state.getOrientationBaseToWorld()).getMappingFromDiffToLocalAngularVelocity();
  }

  KINDR_ASSERT_DOUBLE_MX_EQ(A, Jt_A, 1e-10, msg);
}

TEST_F(KinematicsWorldToFootTest, getTranslationalJacobianTimeDerivativeWorldToFootInWorldFrame) {
  init();

  int legId = 0;
  BranchEnum branch = getModelPtr()->getBranchEnumFromLimbUInt(legId);

  AnymalState state;
  state.setRandom();
  state.setOrientationBaseToWorld(kindr::RotationQuaternionD(EulerAnglesXyz(0.0, -0.5, 0.8).inverted()));
  state.setAngularVelocityBaseInBaseFrame(kindr::LocalAngularVelocityD(0.0, 0.5, 1.0));
  // state.setAngularVelocityBaseInBaseFrame(LocalAngularVelocity());
  getModelPtr()->setState(state, true);

  // store some of the kinematics
  const RotationMatrix rotationWorldToBase = RotationMatrix(getModelPtr()->getOrientationWorldToBody(BodyEnum::BASE));

  const GeneralizedCoordinates generalizedPositionsQuaternion = state.getGeneralizedCoordinates();
  const GeneralizedVelocities generalizedVelocities = state.getGeneralizedVelocities();

  const EulerAnglesXyz eulerAnglesXyz = EulerAnglesXyz(state.getOrientationBaseToWorld());
  // const EulerAnglesXyz eulerAnglesXyz = EulerAnglesXyz(-eulerAnglesXyzTmp.vector());
  const LocalAngularVelocity angularVelocityBaseInBaseFrame = state.getAngularVelocityBaseInBaseFrame();
  const LocalAngularVelocity angularVelocityBaseInWorldFrame = rotationWorldToBase.inverseRotate(angularVelocityBaseInBaseFrame);
  // const EulerAnglesXyzDiff eulerAnglesXyzDiff(eulerAnglesXyz, angularVelocityBaseInBaseFrame);

  EulerAnglesXyzDiff eulerAnglesXyzDiff(eulerAnglesXyz.getMappingFromLocalAngularVelocityToDiff() *
                                        angularVelocityBaseInBaseFrame.vector());

  // get the main body attitude using euler xyz angles
  Eigen::VectorXd generalizedPositionsEulerXyz(18);
  generalizedPositionsEulerXyz.setZero();
  generalizedPositionsEulerXyz.block<3, 1>(0, 0) = state.getPositionWorldToBaseInWorldFrame().toImplementation();
  generalizedPositionsEulerXyz.block<3, 1>(3, 0) = eulerAnglesXyz.vector();
  generalizedPositionsEulerXyz.block<12, 1>(6, 0) = state.getJointPositions().toImplementation();
  Eigen::VectorXd generalizedVelocitiesEulerAnglesXyz = generalizedVelocities;
  generalizedVelocitiesEulerAnglesXyz.block<3, 1>(3, 0) = eulerAnglesXyzDiff.vector();

  // get conversions between euler angles xyz and angular velocity in base frame
  const Eigen::Matrix3d transformOmegaInBaseFrameToEulerAnglesXyzDiff = eulerAnglesXyz.getMappingFromLocalAngularVelocityToDiff();
  //      romo_std::angularVelocityInBaseFrameToEulerAnglesXyzDiff(eulerAnglesXyz.vector());
  Eigen::Matrix3d transformDerivativeOmegaInBaseFrameToEulerAnglesXyzDiff =
      EulerAnglesXyzDiff(eulerAnglesXyzDiff.vector()).getMappingFromLocalAngularVelocityToSecondDiff(eulerAnglesXyz);
  // const Eigen::Matrix3d transformDerivativeOmegaInBaseFrameToEulerAnglesXyzDiff =
  // romo_std::angularVelocityInBaseFrameToEulerAnglesXyzDiffTimeDerivative(
  //          -eulerAnglesXyz.vector(), eulerAnglesXyzDiff.vector());

  // get jacobian time derivative from starleth robot model
  const Eigen::MatrixXd jacobianWorldToLFFootInWorldFrameFromRobotModel =
      getJacobianWorldToLFFootInWorldFrameFromRobotModel(generalizedPositionsEulerXyz);
  const Eigen::MatrixXd jacobianDerivativeWorldToLFFootInWorldFrameFromRobotModel =
      getJacobianDerivativeWorldToLFFootInWorldFrameFromRobotModel(generalizedPositionsEulerXyz, generalizedVelocitiesEulerAnglesXyz);

  // get jacobian from anymal model
  Eigen::MatrixXd jacobianWorldToLFFootInWorldFrameFromAnymalModel(3, 18);
  jacobianWorldToLFFootInWorldFrameFromAnymalModel.setZero();
  getModelPtr()->getJacobianTranslationWorldToBody(jacobianWorldToLFFootInWorldFrameFromAnymalModel, branch, BodyNodeEnum::FOOT,
                                                   CoordinateFrameEnum::WORLD);

  // get jacobian time derivative from anymal model
  Eigen::MatrixXd jacobianTranslationDerivativeWorldToLFFootInWorldFrameFromAnymalModel =
      Eigen::MatrixXd::Zero(3, getModelPtr()->getDofCount());
  getModelPtr()->getJacobianTranslationTimeDerivativeWorldToBody(jacobianTranslationDerivativeWorldToLFFootInWorldFrameFromAnymalModel,
                                                                 branch, BodyNodeEnum::FOOT, CoordinateFrameEnum::WORLD);

  // convert main body orientation from euler angles xyz diff to angular velocity
  Eigen::MatrixXd jacobianDerivativeWorldToLFFootInWorldFrameFromRobotModelConverted =
      jacobianDerivativeWorldToLFFootInWorldFrameFromRobotModel;
  const Eigen::Matrix3d jacobianFromRobotModel_A = jacobianWorldToLFFootInWorldFrameFromRobotModel.block<3, 3>(0, 3);
  const Eigen::Matrix3d jacobianDerivativeFromRobotModel_A =
      jacobianDerivativeWorldToLFFootInWorldFrameFromRobotModelConverted.block<3, 3>(0, 3);
  jacobianDerivativeWorldToLFFootInWorldFrameFromRobotModelConverted.block<3, 3>(0, 3) =
      jacobianFromRobotModel_A * transformDerivativeOmegaInBaseFrameToEulerAnglesXyzDiff +
      jacobianDerivativeFromRobotModel_A * transformOmegaInBaseFrameToEulerAnglesXyzDiff;

  // compare dJ from anymal model (using angular velocity) and dJ from robot_model (originally using euler xyz diff and converted to angular
  // velocity)
  std::string msg = "";
  KINDR_ASSERT_DOUBLE_MX_EQ(jacobianTranslationDerivativeWorldToLFFootInWorldFrameFromAnymalModel,
                            jacobianDerivativeWorldToLFFootInWorldFrameFromRobotModelConverted, 1.0e-3, msg);
}

TEST_F(KinematicsWorldToFootTest, getSpatialJacobianTimeDerivativeWorldToFootInWorldFrame) {
  init();

  int legId = 0;
  BranchEnum branch = getModelPtr()->getBranchEnumFromLimbUInt(legId);

  AnymalState state;
  state.setRandom();
  getModelPtr()->setState(state, true, true);

  // get jacobian time derivative from anymal model
  Eigen::MatrixXd jacobianTranslationDerivativeWorldToLFFootInWorldFrame(3, 18);
  jacobianTranslationDerivativeWorldToLFFootInWorldFrame.setZero();
  getModelPtr()->getJacobianTranslationTimeDerivativeWorldToBody(jacobianTranslationDerivativeWorldToLFFootInWorldFrame, branch,
                                                                 BodyNodeEnum::FOOT, CoordinateFrameEnum::WORLD);

  // get jacobian time derivative from anymal model (spatial jacobian method)
  Eigen::MatrixXd jacobianSpatialDerivativeWorldToLFFootInWorldFrame(6, 18);
  jacobianSpatialDerivativeWorldToLFFootInWorldFrame.setZero();
  getModelPtr()->getJacobianSpatialTimeDerivativeWorldToBody(jacobianSpatialDerivativeWorldToLFFootInWorldFrame, branch, BodyNodeEnum::FOOT,
                                                             CoordinateFrameEnum::WORLD);

  const Eigen::MatrixXd jacobianTranslationDerivativeWorldToLFFootInWorldFrameFromSpatialJacobian =
      jacobianSpatialDerivativeWorldToLFFootInWorldFrame.block<3, 18>(3, 0);

  std::string msg = "";
  KINDR_ASSERT_DOUBLE_MX_EQ(jacobianTranslationDerivativeWorldToLFFootInWorldFrame,
                            jacobianTranslationDerivativeWorldToLFFootInWorldFrameFromSpatialJacobian, 1.0e-10, msg);
}

TEST_F(KinematicsWorldToFootTest, Rotations) {
  initModel("starleth_unit_test", USE_QUATERNION);

  int legId = 0;
  BranchEnum branch = getModelPtr()->getBranchEnumFromLimbUInt(legId);

  AnymalState state;
  state.setRandom();
  getModelPtr()->setState(state, true);

  const Eigen::Matrix3d& orientationWorldToBaseFromModel = getModelPtr()->getOrientationWorldToBody(BodyEnum::BASE);
  const Eigen::Matrix3d orientationWorldToBaseFromState =
      RotationMatrix(getModelPtr()->getState().getOrientationBaseToWorld().inverted()).matrix();
  const Eigen::Matrix3d orientationBaseToWorldFromModel = orientationWorldToBaseFromModel.transpose();
  const Eigen::Matrix3d orientationBaseToWorldFromState = orientationWorldToBaseFromState.transpose();

  const RotationMatrix orientationWorldToBaseFromModelKindr = RotationMatrix(getModelPtr()->getOrientationWorldToBody(BodyEnum::BASE));
  const RotationMatrix orientationWorldToBaseFromStateKindr =
      RotationMatrix(getModelPtr()->getState().getOrientationBaseToWorld().inverted());
  const RotationMatrix orientationBaseToWorldFromModelKindr = orientationWorldToBaseFromModelKindr.inverted();
  const RotationMatrix orientationBaseToWorldFromStateKindr = orientationWorldToBaseFromStateKindr.inverted();

  std::string msg = "";
  KINDR_ASSERT_DOUBLE_MX_EQ(orientationWorldToBaseFromModel, orientationWorldToBaseFromState, 1.0e-10, msg);

  Eigen::MatrixXd jacobianTranslationWorldToLFFootInWorldFrame = Eigen::MatrixXd::Zero(3, 18);
  getModelPtr()->getJacobianTranslationWorldToBody(jacobianTranslationWorldToLFFootInWorldFrame, branch, BodyNodeEnum::FOOT,
                                                   CoordinateFrameEnum::WORLD);

  Eigen::MatrixXd jacobianRotationWorldToLFFootInWorldFrame = Eigen::MatrixXd::Zero(3, 18);
  getModelPtr()->getJacobianRotationWorldToBody(jacobianRotationWorldToLFFootInWorldFrame, branch, BodyNodeEnum::FOOT,
                                                CoordinateFrameEnum::WORLD);

  Eigen::MatrixXd jacobianTranslationWorldToLFFootInBaseFrame = Eigen::MatrixXd::Zero(3, 18);
  getModelPtr()->getJacobianTranslationWorldToBody(jacobianTranslationWorldToLFFootInBaseFrame, branch, BodyNodeEnum::FOOT,
                                                   CoordinateFrameEnum::BASE);

  Eigen::MatrixXd jacobianRotationWorldToLFFootInBaseFrame = Eigen::MatrixXd::Zero(3, 18);
  getModelPtr()->getJacobianRotationWorldToBody(jacobianRotationWorldToLFFootInBaseFrame, branch, BodyNodeEnum::FOOT,
                                                CoordinateFrameEnum::BASE);

  const Eigen::Vector3d B_p_BLF = getModelPtr()->getPositionBodyToBody(BodyEnum::BASE, BodyEnum::LF_FOOT, CoordinateFrameEnum::BASE);
  const Eigen::Vector3d I_p_BLF = getModelPtr()->getPositionBodyToBody(BodyEnum::BASE, BodyEnum::LF_FOOT, CoordinateFrameEnum::WORLD);
  const Eigen::MatrixXd neg_S_B_p_BLF = -kindr::getSkewMatrixFromVector(B_p_BLF);
  const Eigen::MatrixXd neg_S_I_p_BLF = -kindr::getSkewMatrixFromVector(I_p_BLF);

  const Eigen::MatrixXd I_Jp_v = jacobianTranslationWorldToLFFootInWorldFrame.middleCols(0, 3);
  const Eigen::MatrixXd B_Jp_v = jacobianTranslationWorldToLFFootInBaseFrame.middleCols(0, 3);
  const Eigen::MatrixXd I_Jr_v = jacobianRotationWorldToLFFootInWorldFrame.middleCols(0, 3);
  const Eigen::MatrixXd B_Jr_v = jacobianRotationWorldToLFFootInBaseFrame.middleCols(0, 3);

  Eigen::MatrixXd I_Jp_w = jacobianTranslationWorldToLFFootInWorldFrame.middleCols(3, 3);
  if (!USE_QUATERNION) {
    I_Jp_w *= kindr::EulerAnglesZyxD(state.getOrientationBaseToWorld()).getMappingFromLocalAngularVelocityToDiff();
  }
  Eigen::MatrixXd B_Jp_w = jacobianTranslationWorldToLFFootInBaseFrame.middleCols(3, 3);
  if (!USE_QUATERNION) {
    B_Jp_w *= kindr::EulerAnglesZyxD(state.getOrientationBaseToWorld()).getMappingFromLocalAngularVelocityToDiff();
  }
  Eigen::MatrixXd I_Jr_w = jacobianRotationWorldToLFFootInWorldFrame.middleCols(3, 3);
  if (!USE_QUATERNION) {
    I_Jr_w *= kindr::EulerAnglesZyxD(state.getOrientationBaseToWorld()).getMappingFromLocalAngularVelocityToDiff();
  }
  Eigen::MatrixXd B_Jr_w = jacobianRotationWorldToLFFootInBaseFrame.middleCols(3, 3);
  if (!USE_QUATERNION) {
    B_Jr_w *= kindr::EulerAnglesZyxD(state.getOrientationBaseToWorld()).getMappingFromLocalAngularVelocityToDiff();
  }

  const Eigen::MatrixXd expected_I_Jp_v = Eigen::MatrixXd::Identity(3, 3);
  const Eigen::MatrixXd expected_B_Jp_v = orientationWorldToBaseFromModel;
  const Eigen::MatrixXd expected_I_Jr_v = Eigen::MatrixXd::Zero(3, 3);
  const Eigen::MatrixXd expected_B_Jr_v = Eigen::MatrixXd::Zero(3, 3);

  const Eigen::MatrixXd expected_I_Jp_w = orientationWorldToBaseFromModel.transpose() * neg_S_B_p_BLF;
  const Eigen::MatrixXd expected_B_Jp_w = neg_S_B_p_BLF;
  const Eigen::MatrixXd expected_I_Jr_w = orientationBaseToWorldFromModel;
  const Eigen::MatrixXd expected_B_Jr_w = Eigen::MatrixXd::Identity(3, 3);

  KINDR_ASSERT_DOUBLE_MX_EQ(I_Jp_v, expected_I_Jp_v, 1.0e-10, msg);
  KINDR_ASSERT_DOUBLE_MX_EQ(B_Jp_v, expected_B_Jp_v, 1.0e-10, msg);
  KINDR_ASSERT_DOUBLE_MX_EQ(I_Jr_v, expected_I_Jr_v, 1.0e-10, msg);
  KINDR_ASSERT_DOUBLE_MX_EQ(B_Jr_v, expected_B_Jr_v, 1.0e-10, msg);

  KINDR_ASSERT_DOUBLE_MX_EQ(I_Jp_w, expected_I_Jp_w, 1.0e-10, msg);
  KINDR_ASSERT_DOUBLE_MX_EQ(B_Jp_w, expected_B_Jp_w, 1.0e-10, msg);
  KINDR_ASSERT_DOUBLE_MX_EQ(I_Jr_w, expected_I_Jr_w, 1.0e-10, msg);
  KINDR_ASSERT_DOUBLE_MX_EQ(B_Jr_w, expected_B_Jr_w, 1.0e-10, msg);
}

TEST_F(KinematicsWorldToFootTest, getLinearVelocityFootInWorldFrame) {
  initModel("starleth_unit_test", USE_QUATERNION);

  for (int i = 0; i < 4; i++) {  // todo: do not hardcode limbId
    BranchEnum branch = getModelPtr()->getBranchEnumFromLimbUInt(i);

    AnymalState state;  // = Eigen::VectorXd::Random(getModelPtr()->getDofCount());
    state.setRandom();
    getModelPtr()->setState(state, true, true);

    std::chrono::time_point<std::chrono::steady_clock> start, end;
    int64_t elapsedTimeNSecs;
    start = std::chrono::steady_clock::now();

    //-- Compute expected linear velocity by Jacobian
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, getModelPtr()->getDofCount());
    getModelPtr()->getJacobianTranslationWorldToBody(jacobian, branch, BodyNodeEnum::FOOT, CoordinateFrameEnum::WORLD);
    Eigen::VectorXd genVel = state.getGeneralizedVelocities();
    if (!USE_QUATERNION) {
      genVel.segment<3>(3) =
          kindr::EulerAnglesZyxDiffD(kindr::EulerAnglesZyxD(state.getOrientationBaseToWorld()), state.getAngularVelocityBaseInBaseFrame())
              .vector();
    }
    Eigen::Vector3d expectedVelocity = jacobian * genVel;
    //--
    end = std::chrono::steady_clock::now();
    //    std::cout << "Time: Jacobian method: (nsec): " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() <<
    //    std::endl;

    start = std::chrono::steady_clock::now();
    Eigen::Vector3d computedVelocity = getModelPtr()->getLinearVelocityBody(branch, BodyNodeEnum::FOOT, CoordinateFrameEnum::WORLD);
    end = std::chrono::steady_clock::now();
    //    std::cout << "Time: RBDL (nsec): " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << std::endl;

    std::string msg = "getLinearVelocityFootInWorldFrame for limb " + std::to_string(i);
    KINDR_ASSERT_DOUBLE_MX_EQ(expectedVelocity, computedVelocity, 1.0e-5, msg);
  }
}
