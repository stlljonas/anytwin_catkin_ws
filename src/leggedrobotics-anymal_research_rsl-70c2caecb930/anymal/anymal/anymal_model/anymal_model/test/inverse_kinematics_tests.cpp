/*!
 * @file    invesrse_kinematics_tests.cpp
 * @author  Christian Gehring, Dario Bellicoso, Peter Fankhauser
 * @date    Sep, 2015
 */

// test
#include <gtest/gtest.h>
#include <kindr/common/gtest_eigen.hpp>

// anymal model
#include "anymal_model/AnymalModel.hpp"

// anymal description
#include "anymal_description/AnymalDescription.hpp"

// boost
#include <boost/filesystem.hpp>

// fixture
#include "TestAnymalModel.hpp"

using anymal_model::AD;

Eigen::Vector3d getJointPositionFromFootPositionForStarlETH(const Eigen::Vector3d& rFoot2Hip_CSh, int iLeg,
                                                            anymal_model::AnymalModel* anymalModel) {
  //  analytical inverse kinematics in the local CS (fixed to the main body orientation
  //  at the hip joint)
  Eigen::Vector3d q;
  Eigen::Vector3d q0(0, 0, 0);

  // HAA
  if (iLeg == 0 || iLeg == 2) {
    q0(0) = 0.2;
  } else {
    q0(0) = -0.2;
  }

  bool isLegNormalBent = true;
  switch (iLeg) {
    case 0:
    case 1:
      isLegNormalBent = true;
      break;
    case 2:
    case 3:
      isLegNormalBent = false;
      break;
  }

  // HFE and KFE
  if (isLegNormalBent) {
    q0(1) = 0.7;
    q0(2) = -1.4;
  } else {
    q0(1) = -0.7;
    q0(2) = 1.4;
  }

  double qHAA, qHFE, qKFE;
  double d;

  double lH = anymalModel->getParameters().getPositionHipToThighInHipFrameForLeg(AD::LimbEnum::LF_LEG).z();
  double lT = anymalModel->getParameters().getPositionThighToShankInThighFrameForLeg(AD::LimbEnum::LF_LEG).z();
  double lS = anymalModel->getParameters().getPositionShankToFootInShankFrameForLeg(AD::LimbEnum::LF_LEG).z();

  //  std::cout << "lh: " << lH << "lT: " << lT << "lS: " << lS << std::endl;

  qHAA = std::atan(static_cast<double>(rFoot2Hip_CSh(1) / (-rFoot2Hip_CSh(2))));
  // double tmp = std::fabs(static_cast<double>(rFoot2Hip_CSh(2))) - std::fabs(lH * cos(qHAA));

  Eigen::Vector3d r_tmp;
  r_tmp(0) = rFoot2Hip_CSh(0);
  r_tmp(1) = rFoot2Hip_CSh(1) + lH * sin(qHAA);
  r_tmp(2) = rFoot2Hip_CSh(2) - lH * cos(qHAA);

  d = r_tmp.norm();
  // d = sqrt(rFoot2Hip_CSh(0)*rFoot2Hip_CSh(0)+rFoot2Hip_CSh(1)*rFoot2Hip_CSh(1)+tmp*tmp);

  // If d is to large project the footpoint onto the sphere
  if (d > std::fabs(lT) + std::fabs(lS)) {
    //    cout << "No solution for inverse kinematic -> taking nearest point!" << endl;
    r_tmp = r_tmp / d * (std::fabs(lT) + std::fabs(lS));
    d = std::fabs(lT) + std::fabs(lS);
  }
  double cKFE = (d * d - lT * lT - lS * lS) / (2.0 * std::fabs(lT) * std::fabs(lS));
  if (abs(cKFE) >= 1) {
    cKFE = cKFE / std::fabs(cKFE);
  }
  qKFE = std::acos(cKFE);

  // we have to find for the correct solution since now angles in both
  // directions are possible
  if (std::fabs(qKFE - static_cast<double>(q0(2))) > std::fabs(qKFE + static_cast<double>(q0(2)))) {
    qKFE = -qKFE;
  }
  qHFE = M_PI / 2.0 -
         (std::atan2(std::sqrt(d * d - static_cast<double>(r_tmp(0)) * static_cast<double>(r_tmp(0))), (static_cast<double>(r_tmp(0)))) +
          atan2(lS * sin(qKFE), (lT + lS * cos(qKFE))));

  q(0) = qHAA;
  q(1) = qHFE;
  q(2) = qKFE;
  return q;
}

// TODO(dbellicoso): reenable this test when the urdf description for StarlETH is updated.
// TEST(InverseKintematicsTest, starleth) { // NOLINT
//  //-- Load anymal for testing
//  boost::filesystem::path filePath(__FILE__);
//  std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/starleth_unit_test.urdf"};
//  AnymalModel anymalModel;
//  ASSERT_TRUE(anymalModel.initializeFromUrdf(path));
//  AnymalState state;
//  state.setRandom();
//  anymalModel.setState(state);
//  //--
//
//  // test for all four legs
//  for (int iLeg=0; iLeg<4; iLeg++) {
//
//    Eigen::Vector3d positionBaseToHipInBaseFrame;
//    anymalModel.getPositionBodyToBody(positionBaseToHipInBaseFrame,
//                                         BodyEnum::BASE,
//                                         static_cast<BranchEnum>(iLeg+1),
//                                         BodyNodeEnum::HIP,
//                                         CoordinateFrameEnum::BASE);
//
//
//    // this is the input:
//    const Eigen::Vector3d positionBaseToFootInBaseFrame = positionBaseToHipInBaseFrame + Eigen::Vector3d(0.05, 0.07, -0.2);
//
//    //-- Compute inverse kinematics with old method from robot model for StarlETH
//    const Eigen::Vector3d rFoot2Hip_CSh = positionBaseToHipInBaseFrame - positionBaseToFootInBaseFrame;
//    const Eigen::Vector3d expectedJointPositions = getJointPositionFromFootPositionForStarlETH(rFoot2Hip_CSh, iLeg, &anymalModel);
//    //--
//
//    //-- Compute inverse kinematics with anymal model
//    Eigen::Vector3d computedJointPositions;
//    anymalModel.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(computedJointPositions,
//                                                                          positionBaseToFootInBaseFrame,
//                                                                          iLeg);
//    //--
//    std::string msg = "Inverse kinematics for leg " + std::to_string(iLeg);
//    KINDR_ASSERT_DOUBLE_MX_EQ(expectedJointPositions, computedJointPositions, 1.0e-12, msg);
//
//  }
//}

class InverseKinematicsTest : public TestAnymalModel {};

TEST(InverseKinematicsTest, anymal) {  // NOLINT
  //-- Load anymal for testing
  boost::filesystem::path filePath(__FILE__);
  std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/anymal_minimal.urdf"};
  anymal_model::AnymalModel anymalModel;
  ASSERT_TRUE(anymalModel.initializeFromUrdf(path));
  //--

  // test for all four legs
  for (const auto limbKey : AD::getLimbKeys()) {
    const auto limbId = limbKey.getId();
    const auto limbEnum = limbKey.getEnum();
    const auto branchEnum = AD::mapEnums<AD::BranchEnum>(limbEnum);

    Eigen::Vector3d positionBaseToHipInBaseFrame;
    anymalModel.getPositionBodyToBody(positionBaseToHipInBaseFrame, AD::BodyEnum::BASE, branchEnum, AD::BodyNodeEnum::HIP,
                                      AD::CoordinateFrameEnum::BASE);

    // this is the input:
    for (double ds = 0.0; ds <= 1.0; ds += 0.01) {
      const Eigen::Vector3d positionBaseToFootInBaseFrame =
          positionBaseToHipInBaseFrame + Eigen::Vector3d(0.05, 0.07 + ds * 0.05, -0.2 - ds * 0.2);

      //-- Compute inverse kinematics with anymal model
      Eigen::Vector3d computedJointPositions;
      anymalModel.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(computedJointPositions, positionBaseToFootInBaseFrame, limbId);

      anymal_model::AnymalState anymalState = anymalModel.getState();
      anymal_model::JointPositions jointPositions = anymalState.getJointPositions();
      jointPositions.toImplementation().segment<AD::getNumDofLimb()>(AD::getLimbStartIndexInJ(limbEnum)) = computedJointPositions;
      anymalState.setJointPositions(jointPositions);
      anymalModel.setState(anymalState, true, false, false);

      Eigen::Vector3d positionBaseToFootInBaseFrameForwardKinematics =
          anymalModel.getPositionBodyToBody(AD::BodyEnum::BASE, branchEnum, AD::BodyNodeEnum::FOOT, AD::CoordinateFrameEnum::BASE);
      //--

      std::string msg = "Inverse kinematics for leg " + std::string{AD::mapKeyEnumToKeyName(limbEnum)};
      KINDR_ASSERT_DOUBLE_MX_EQ(positionBaseToFootInBaseFrame, positionBaseToFootInBaseFrameForwardKinematics, 1.0e-10, msg);
    }
  }
}

TEST(InverseKinematicsTest, anymal_joints) {  // NOLINT
  //-- Load anymal for testing
  boost::filesystem::path filePath(__FILE__);
  std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/anymal_minimal.urdf"};
  anymal_model::AnymalModel anymalModel;
  ASSERT_TRUE(anymalModel.initializeFromUrdf(path));
  //--

  // test for all four legs
  const auto limbEnum = AD::LimbEnum::LF_LEG;
  const auto limbId = AD::mapKeyEnumToKeyId(limbEnum);
  const auto branchEnum = AD::mapEnums<AD::BranchEnum>(limbEnum);

  Eigen::Vector3d positionBaseToHipInBaseFrame;
  anymalModel.getPositionBodyToBody(positionBaseToHipInBaseFrame, AD::BodyEnum::BASE, branchEnum, AD::BodyNodeEnum::HIP,
                                    AD::CoordinateFrameEnum::BASE);

  // this is the input:
  for (double ds = 0.0; ds <= 1.0; ds += 0.005) {
    Eigen::Vector3d joints = Eigen::Vector3d(0.05, 0.0, -M_PI_4 - ds * M_PI_4 / 2.0);

    anymal_model::AnymalState anymalState = anymalModel.getState();
    anymal_model::JointPositions jointPositions = anymalState.getJointPositions();
    jointPositions.toImplementation().segment<AD::getNumDofLimb()>(AD::getLimbStartIndexInJ(limbEnum)) = joints;
    anymalState.setJointPositions(jointPositions);
    anymalModel.setState(anymalState, true, false, false);

    Eigen::Vector3d positionBaseToFootInBaseFrame =
        anymalModel.getPositionBodyToBody(AD::BodyEnum::BASE, branchEnum, AD::BodyNodeEnum::FOOT, AD::CoordinateFrameEnum::BASE);

    //-- Compute inverse kinematics with anymal model
    Eigen::Vector3d computedJointPositions;
    anymalModel.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(computedJointPositions, positionBaseToFootInBaseFrame, limbId);
    //--

    std::string msg = std::string{"Inverse kinematics for leg "} + std::string{AD::mapKeyEnumToKeyName(limbEnum)};
    KINDR_ASSERT_DOUBLE_MX_EQ(joints, computedJointPositions, 1.0e-8, msg);
  }
}

TEST(InverseKinematicsTest, anymal_o_config1) {  // NOLINT
  //-- Load anymal for testing
  boost::filesystem::path filePath(__FILE__);
  std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/anymal_minimal.urdf"};
  anymal_model::AnymalModel anymalModel;
  ASSERT_TRUE(anymalModel.initializeFromUrdf(path));
  //--

  const auto limbEnum = AD::LimbEnum::LF_LEG;
  const auto limbId = AD::mapKeyEnumToKeyId(limbEnum);
  const auto branchEnum = AD::mapEnums<AD::BranchEnum>(limbEnum);

  Eigen::Vector3d positionBaseToFootInBaseFrame = Eigen::Vector3d(0.728159, 0.220873, -0.023272);

  //-- Compute inverse kinematics with anymal model
  Eigen::Vector3d computedJointPositions;
  anymalModel.setLegConfigurations(anymal_model::LegConfigurationOO());
  anymalModel.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(computedJointPositions, positionBaseToFootInBaseFrame, limbId);
  //--

  //-- Use forward kinematics to check result
  anymal_model::AnymalState anymalState = anymalModel.getState();
  anymal_model::JointPositions jointPositions = anymalState.getJointPositions();
  jointPositions.toImplementation().segment<AD::getNumDofLimb()>(AD::getLimbStartIndexInJ(limbEnum)) = computedJointPositions;
  anymalState.setJointPositions(jointPositions);
  anymalModel.setState(anymalState, true, false, false);

  Eigen::Vector3d positionBaseToFootInBaseFrameForwardKinematics =
      anymalModel.getPositionBodyToBody(AD::BodyEnum::BASE, branchEnum, AD::BodyNodeEnum::FOOT, AD::CoordinateFrameEnum::BASE);
  //--

  std::string msg = "Inverse kinematics for leg " + std::string{AD::mapKeyEnumToKeyName(limbEnum)};
  KINDR_ASSERT_DOUBLE_MX_EQ(positionBaseToFootInBaseFrame, positionBaseToFootInBaseFrameForwardKinematics, 1.0e-10, msg);
}

TEST(InverseKinematicsTest, anymal_o_config2) {  // NOLINT
  //-- Load anymal for testing
  boost::filesystem::path filePath(__FILE__);
  std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/anymal_minimal.urdf"};
  anymal_model::AnymalModel anymalModel;
  ASSERT_TRUE(anymalModel.initializeFromUrdf(path));
  //--

  const auto limbEnum = AD::LimbEnum::LF_LEG;
  const auto limbId = AD::mapKeyEnumToKeyId(limbEnum);
  const auto branchEnum = AD::mapEnums<AD::BranchEnum>(limbEnum);
  Eigen::Vector3d positionBaseToFootInBaseFrame = Eigen::Vector3d(0.7, 0.3, 0.1);

  //-- Compute inverse kinematics with anymal model
  Eigen::Vector3d computedJointPositions;
  anymalModel.setLegConfigurations(anymal_model::LegConfigurationOO());
  anymalModel.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(computedJointPositions, positionBaseToFootInBaseFrame, limbId);
  //--

  //-- Use forward kinematics to check result
  anymal_model::AnymalState anymalState = anymalModel.getState();
  anymal_model::JointPositions jointPositions = anymalState.getJointPositions();
  jointPositions.toImplementation().segment<AD::getNumDofLimb()>(AD::getLimbStartIndexInJ(limbEnum)) = computedJointPositions;
  anymalState.setJointPositions(jointPositions);
  anymalModel.setState(anymalState, true, false, false);

  Eigen::Vector3d positionBaseToFootInBaseFrameForwardKinematics =
      anymalModel.getPositionBodyToBody(AD::BodyEnum::BASE, branchEnum, AD::BodyNodeEnum::FOOT, AD::CoordinateFrameEnum::BASE);
  //--

  std::string msg = "Inverse kinematics for leg " + std::string{AD::mapKeyEnumToKeyName(limbEnum)};
  KINDR_ASSERT_DOUBLE_MX_EQ(positionBaseToFootInBaseFrame, positionBaseToFootInBaseFrameForwardKinematics, 1.0e-10, msg);
}

TEST(InverseKinematicsTest, iterativeBigValues) {  // NOLINT
  //-- Load anymal for testing
  boost::filesystem::path filePath(__FILE__);
  std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/anymal_minimal.urdf"};
  anymal_model::AnymalModel anymalModel;
  ASSERT_TRUE(anymalModel.initializeFromUrdf(path));
  anymal_model::AnymalState anymalState = anymalModel.getState();
  anymalState.setPositionWorldToBaseInWorldFrame(anymal_model::Position(1.0, 2.0, 3.0));
  anymalModel.setState(anymalState, true, false, false);
  //--
  // test for all four legs
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto limbEnum = AD::mapEnums<AD::LimbEnum>(contactEnum);
    // const auto limbId = AD::mapKeyEnumToKeyId(limbEnum);
    const auto branchEnum = AD::mapEnums<AD::BranchEnum>(limbEnum);

    Eigen::Vector3d positionBaseToHipInBaseFrame;
    positionBaseToHipInBaseFrame.setZero();
    anymalModel.getPositionBodyToBody(positionBaseToHipInBaseFrame, AD::BodyEnum::BASE, branchEnum, AD::BodyNodeEnum::HIP,
                                      AD::CoordinateFrameEnum::BASE);

    // this is the input:
    for (double ds = 0.0; ds <= 1.0; ds += 0.01) {
      Eigen::Vector3d positionBaseToFootInBaseFrame =
          positionBaseToHipInBaseFrame + Eigen::Vector3d(0.05, 0.07 + ds * 0.05, -0.2 - ds * 0.2);

      //-- Compute inverse kinematics with anymal model
      Eigen::Vector3d computedJointPositions;
      anymalModel.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(computedJointPositions, positionBaseToFootInBaseFrame, limbEnum);

      anymal_model::AnymalState anymalState = anymalModel.getState();
      anymal_model::JointPositions jointPositions;
      jointPositions.toImplementation().segment<AD::getNumDofLimb()>(AD::getLimbStartIndexInJ(limbEnum)) =
          computedJointPositions.array() + 0.2 * Eigen::Array3d::Random();
      anymalState.setJointPositions(jointPositions);
      anymalModel.setState(anymalState, true, false, false);

      // Compare to using iterative method.
      Eigen::VectorXd computedJointPositionsIteratively;
      anymalModel.getLimbJointPositionsFromLimbEnumIteratively(computedJointPositionsIteratively, positionBaseToFootInBaseFrame, limbEnum);

      kindr::assertNear(computedJointPositions, computedJointPositionsIteratively, 1e-3, KINDR_SOURCE_FILE_POS);
    }
  }
}

TEST(InverseKinematicsTest, iterativeSmallValues) {  // NOLINT
  //-- Load anymal for testing
  boost::filesystem::path filePath(__FILE__);
  std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/anymal_minimal.urdf"};
  anymal_model::AnymalModel anymalModel;
  ASSERT_TRUE(anymalModel.initializeFromUrdf(path));
  anymal_model::AnymalState anymalState = anymalModel.getState();
  anymalState.setPositionWorldToBaseInWorldFrame(anymal_model::Position(1.0, 2.0, 3.0));
  anymalModel.setState(anymalState, true, false, false);
  //--
  // test for all four legs
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto limbEnum = AD::mapEnums<AD::LimbEnum>(contactEnum);
    // const auto limbId = AD::mapKeyEnumToKeyId(limbEnum);
    const auto branchEnum = AD::mapEnums<AD::BranchEnum>(limbEnum);

    Eigen::Vector3d positionBaseToHipInBaseFrame;
    positionBaseToHipInBaseFrame.setZero();
    anymalModel.getPositionBodyToBody(positionBaseToHipInBaseFrame, AD::BodyEnum::BASE, branchEnum, AD::BodyNodeEnum::HIP,
                                      AD::CoordinateFrameEnum::BASE);

    // this is the input:
    for (double ds = 0.0; ds <= 1.0; ds += 0.01) {
      Eigen::Vector3d positionBaseToFootInBaseFrame =
          positionBaseToHipInBaseFrame + Eigen::Vector3d(0.05, 0.07 + ds * 0.05, -0.2 - ds * 0.2);

      //-- Compute inverse kinematics with anymal model
      Eigen::Vector3d computedJointPositions;
      anymalModel.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(computedJointPositions, positionBaseToFootInBaseFrame, limbEnum);

      anymal_model::AnymalState anymalState = anymalModel.getState();
      anymal_model::JointPositions jointPositions;
      jointPositions.toImplementation().segment<AD::getNumDofLimb()>(AD::getLimbStartIndexInJ(limbEnum)) =
          computedJointPositions.array() + 1e-5 * Eigen::Array3d::Random();
      anymalState.setJointPositions(jointPositions);
      anymalModel.setState(anymalState, true, false, false);

      // Compare to using iterative method.
      Eigen::VectorXd computedJointPositionsIteratively;
      anymalModel.getLimbJointPositionsFromLimbEnumIteratively(computedJointPositionsIteratively, positionBaseToFootInBaseFrame, limbEnum);

      kindr::assertNear(computedJointPositions, computedJointPositionsIteratively, 1e-3, KINDR_SOURCE_FILE_POS);
    }
  }
}

TEST(InverseKinematicsTest, iterativeDebugging) {  // NOLINT
  //-- Load anymal for testing
  boost::filesystem::path filePath(__FILE__);
  std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/anymal_minimal.urdf"};
  anymal_model::AnymalModel anymalModel;
  ASSERT_TRUE(anymalModel.initializeFromUrdf(path));
  anymal_model::AnymalState anymalState = anymalModel.getState();
  anymalState.setPositionWorldToBaseInWorldFrame(anymal_model::Position(-0.033673, 0.044244, 0.390535));
  anymalState.setOrientationBaseToWorld(anymal_model::RotationQuaternion(-0.007645, -0.002148, -0.000663, 0.999968).inverted());
  anymal_model::JointPositions jointPositions;
  jointPositions << 0.000789, 0.782362, -1.703176, -0.186474, 0.688428, -1.531549, 0.000611, -0.977320, 1.746492, -0.191229, -0.871064,
      1.571839;
  anymalState.setJointPositions(jointPositions);
  anymalModel.setState(anymalState, true, false, false);
  //--
  // test leg
  Eigen::Vector3d positionBaseToFootInBaseFrame(0.379592, -0.292022, -0.371704);

  //-- Compute inverse kinematics with anymal model
  Eigen::Vector3d computedJointPositions;
  anymalModel.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(computedJointPositions, positionBaseToFootInBaseFrame,
                                                                     AD::LimbEnum::RF_LEG);

  // Compare to using iterative method.
  Eigen::VectorXd computedJointPositionsIteratively;
  anymalModel.getLimbJointPositionsFromLimbEnumIteratively(computedJointPositionsIteratively, positionBaseToFootInBaseFrame,
                                                           AD::LimbEnum::RF_LEG);

  kindr::assertNear(computedJointPositions, computedJointPositionsIteratively, 1e-3, KINDR_SOURCE_FILE_POS);
}
