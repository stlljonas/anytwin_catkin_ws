/**
 * @authors     Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief       Test for the LegKinematicParameters and AnalyticInverseKinematics class
 */

#include <gtest/gtest.h>
#include <cmath>

#include <kindr/common/gtest_eigen.hpp>

#include "analytical_inverse_kinematics/AnalyticalInverseKinematics.hpp"
#include "analytical_inverse_kinematics/LegKinematicParameters.hpp"

class TestLegKinematicParameters : public ::testing::Test {
 protected:
  using Position = analytical_inverse_kinematics::Position;
  using EulerAnglesZyx = analytical_inverse_kinematics::EulerAnglesZyx;

  void SetUp() override {
    parameters_.setPositionBaseToHipInBaseFrame(positionBaseToHipInBaseFrame_);
    parameters_.setPositionHipToThighInHipFrame(positionHipToThighInHipFrame_);
    parameters_.setPositionThighToShankInThighFrame(positionThighToShankInThighFrame_);
    parameters_.setPositionShankToFootInShankFrame(positionShankToFootInShankFrame_);

    parameters_.setOrientationHipToBase(orientationHipToBase_);
    parameters_.setOrientationThighToHip(orientationThighToHip_);
    parameters_.setOrientationShankToThigh(orientationShankToThigh_);
    parameters_.setOrientationFootToShank(orientationFootToShank_);

    parameters_.initialize();
  }

  void TearDown() override {}

  // Leg base params
  const double hipLength_ = 0.1;
  const double thighLength_ = 0.5;
  const double shankLength_ = 0.5;

  Position positionBaseToHipInBaseFrame_ = Position::Zero();
  Position positionHipToThighInHipFrame_ = Position(0.0, hipLength_, 0.0);
  Position positionThighToShankInThighFrame_ = Position(0.0, 0.0, -thighLength_);
  Position positionShankToFootInShankFrame_ = Position(0.0, 0.0, -shankLength_);

  EulerAnglesZyx orientationHipToBase_ = EulerAnglesZyx::Identity();
  EulerAnglesZyx orientationThighToHip_ = EulerAnglesZyx(M_PI_2, 0.0, 0.0).setUnique();
  EulerAnglesZyx orientationShankToThigh_ = EulerAnglesZyx::Identity();
  EulerAnglesZyx orientationFootToShank_ = EulerAnglesZyx::Identity();

  // Leg derived params
  Position defaultFootPosInHipFrame_ = Position(0.0, hipLength_, -(thighLength_ + shankLength_));

  analytical_inverse_kinematics::LegKinematicParameters parameters_;
  analytical_inverse_kinematics::AnalyticalInverseKinematics inverseKinematics_;
};

/* Test leg parameters */
TEST_F(TestLegKinematicParameters, footPosInitialization) {  // NOLINT
  parameters_.printParameters();
  Position defaultFootPos(0.0, hipLength_, -(thighLength_ + shankLength_));
  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(defaultFootPos.toImplementation(),
                                    parameters_.getDefaultPositionHipToFootInBaseFrame().toImplementation(), 1e-6, 0.0, "");
}

TEST_F(TestLegKinematicParameters, trigCoefficientsKFE) {  // NOLINT
  double aKFE = 0.0;
  double bKFE = 2.0 * thighLength_ * shankLength_;
  double cKFE = pow(thighLength_, 2) + pow(shankLength_, 2);
  ASSERT_DOUBLE_EQ(aKFE, parameters_.getTrigonometricEquationCoefficientKfeA());
  ASSERT_DOUBLE_EQ(bKFE, parameters_.getTrigonometricEquationCoefficientKfeB());
  ASSERT_DOUBLE_EQ(cKFE, parameters_.getTrigonometricEquationCoefficientKfeC());
}

TEST_F(TestLegKinematicParameters, trigCoefficientsHFE) {  // NOLINT
  double qKFE = M_PI_2;
  double aHFE = 0.0;
  double bHFE = 0.0;
  double cHFE = pow(hipLength_, 2) + pow(thighLength_, 2) + pow(shankLength_, 2);
  ASSERT_DOUBLE_EQ(aHFE, parameters_.getTrigonometricEquationCoefficientHfeA(qKFE));
  ASSERT_DOUBLE_EQ(bHFE, parameters_.getTrigonometricEquationCoefficientHfeB(qKFE));
  ASSERT_DOUBLE_EQ(cHFE, parameters_.getTrigonometricEquationCoefficientHfeC(qKFE));
}

/* Test inverse kinematics */
TEST_F(TestLegKinematicParameters, ikComputationZero) {  // NOLINT
  Eigen::Vector3d jointPositions;
  ASSERT_TRUE(inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      jointPositions, defaultFootPosInHipFrame_.toImplementation(), parameters_, false, false));
  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(Eigen::Vector3d::Zero(), jointPositions, 1e-6, 0.0, "");
}

TEST_F(TestLegKinematicParameters, ikComputationFootOnThigh) {  // NOLINT
  Position foldedFootPos(0.0, hipLength_, 0.0);
  Eigen::Vector3d jointPositions;
  ASSERT_TRUE(inverseKinematics_.getLimbJointPositionsFromPositionBaseToFootInBaseFrame(jointPositions, foldedFootPos.toImplementation(),
                                                                                        parameters_, false, false));
  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(Eigen::Vector3d(0.0, 0.0, -M_PI), jointPositions, 1e-6, 0.0, "");
}