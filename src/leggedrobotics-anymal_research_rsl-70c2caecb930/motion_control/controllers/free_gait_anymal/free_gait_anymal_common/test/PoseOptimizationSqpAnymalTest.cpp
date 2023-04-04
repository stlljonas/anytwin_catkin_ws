/*
 * PoseOptimizationSqpInteractiveTest.cpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_anymal_common/free_gait_anymal_common_test/PoseOptimizationHelper.hpp"

#include <kindr/common/gtest_eigen.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace free_gait;

class PoseOptimizationSqpTest : public ::testing::Test {
 protected:
  static void SetUpTestCase()
  {
    sleepDurationBetweenTests_ = new ros::Duration(0.05);
    optimizationHelper_ = new PoseOptimizationHelper();
    optimizationHelper_->setSleepDurationForOptimizationStep(ros::Duration(0.0));
    ros::Duration(4.0).sleep();  // Wait for RViz.
  }

  static void TearDownTestCase()
  {
    delete sleepDurationBetweenTests_;
    delete optimizationHelper_;
    optimizationHelper_ = NULL;
  }

  static PoseOptimizationHelper* optimizationHelper_;
  static ros::Duration* sleepDurationBetweenTests_;
};

ros::Duration* PoseOptimizationSqpTest::sleepDurationBetweenTests_ = NULL;
PoseOptimizationHelper* PoseOptimizationSqpTest::optimizationHelper_ = NULL;


TEST_F(PoseOptimizationSqpTest, SquareUpAtOrigin)
{
  const Stance footPositions({
    {LimbEnum::LF_LEG, Position(0.3, 0.2, 0.0)},
    {LimbEnum::RF_LEG, Position(0.3, -0.2, 0.0)},
    {LimbEnum::LH_LEG, Position(-0.3, 0.2, 0.0)},
    {LimbEnum::RH_LEG, Position(-0.3, -0.2, 0.0)} });
  optimizationHelper_->setFootholdsToReach(footPositions);
  optimizationHelper_->setFootholdsInSupport(footPositions);

  Pose result;
  ASSERT_TRUE(optimizationHelper_->optimize(result));

  Eigen::Vector3d expectedPosition(0.0, 0.0, 0.48);
  RotationMatrix expectedOrientation; // Identity.

  kindr::assertNear(expectedOrientation.matrix(), RotationMatrix(result.getRotation()).matrix(), 1e-2, KINDR_SOURCE_FILE_POS);
  kindr::assertNear(expectedPosition, result.getPosition().vector(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST_F(PoseOptimizationSqpTest, SquareUpAtPositions)
{
  for (double x = 0.0, yaw = 0.0; x < 3.0; x += 0.02, yaw += 0.03) {
    Position position(x, 0.0, 0.0);
    RotationQuaternion orientation(EulerAnglesZyx(yaw, 0.0, 0.0));
    Pose pose(position, orientation);
    Stance footPositions;
    footPositions[LimbEnum::LF_LEG] = pose.transform(Position(0.3, 0.2, 0.0));
    footPositions[LimbEnum::RF_LEG] = pose.transform(Position(0.3, -0.2, 0.0));
    footPositions[LimbEnum::LH_LEG] = pose.transform(Position(-0.3, 0.2, 0.0));
    footPositions[LimbEnum::RH_LEG] = pose.transform(Position(-0.3, -0.2, 0.0));
    optimizationHelper_->setFootholdsToReach(footPositions);
    optimizationHelper_->setFootholdsInSupport(footPositions);

    Pose result;
    result.setIdentity();
    EXPECT_TRUE(optimizationHelper_->optimize(result));

    Position expectedPosition(pose.getPosition());
    expectedPosition.z() += 0.48;
    EXPECT_TRUE(pose.getRotation().isNear(result.getRotation(), 1e-2));
    kindr::assertNear(expectedPosition.vector(), result.getPosition().vector(), 1e-3, KINDR_SOURCE_FILE_POS);
    sleepDurationBetweenTests_->sleep();
  }
}

TEST_F(PoseOptimizationSqpTest, LiftAndLowerOneLeg)
{
  Stance footholdsToReach({
    {LimbEnum::LF_LEG, Position(1.2, 0.2, 0.0)},
    {LimbEnum::RF_LEG, Position(1.2, -0.2, 0.0)},
    {LimbEnum::LH_LEG, Position(0.6, 0.2, 0.0)},
    {LimbEnum::RH_LEG, Position(0.6, -0.2, 0.0)} });

  Stance footholdsInSupport(footholdsToReach);
  footholdsInSupport.erase(LimbEnum::LH_LEG);
  optimizationHelper_->setFootholdsInSupport(footholdsInSupport, 0.05);

  // Lift foot.
  for (double footHeight = 0.0; footHeight < 0.7; footHeight += 0.02) {
    footholdsToReach[LimbEnum::LH_LEG].z() = footHeight;
    optimizationHelper_->setFootholdsToReach(footholdsToReach);
    Pose result;
    EXPECT_TRUE(optimizationHelper_->optimize(result));
    sleepDurationBetweenTests_->sleep();
  }

  // Lower foot.
  for (double footHeight = 0.0; footHeight > -0.3; footHeight -= 0.02) {
    footholdsToReach[LimbEnum::LH_LEG].z() = footHeight;
    optimizationHelper_->setFootholdsToReach(footholdsToReach);
    Pose result;
    EXPECT_TRUE(optimizationHelper_->optimize(result));
    sleepDurationBetweenTests_->sleep();
  }
}

TEST_F(PoseOptimizationSqpTest, SlideForwardAndBackwardOneLeg)
{
  Stance footholdsToReach({
    {LimbEnum::LF_LEG, Position(1.2, 0.2, 0.0)},
    {LimbEnum::RF_LEG, Position(1.2, -0.2, 0.0)},
    {LimbEnum::LH_LEG, Position(0.6, 0.2, 0.0)},
    {LimbEnum::RH_LEG, Position(0.6, -0.2, 0.0)} });

  Stance footholdsInSupport(footholdsToReach);
  footholdsInSupport.erase(LimbEnum::LH_LEG);
  optimizationHelper_->setFootholdsInSupport(footholdsInSupport, 0.05);

  // Slide forward.
  for (double footPositionX = 0.6; footPositionX < 1.0; footPositionX += 0.02) {
    footholdsToReach[LimbEnum::LH_LEG].x() = footPositionX;
    optimizationHelper_->setFootholdsToReach(footholdsToReach);
    Pose result;
    EXPECT_TRUE(optimizationHelper_->optimize(result));
    sleepDurationBetweenTests_->sleep();
  }

  // Slide backward.
  for (double footPositionX = 0.6; footPositionX > 0.1; footPositionX -= 0.02) {
    footholdsToReach[LimbEnum::LH_LEG].x() = footPositionX;
    optimizationHelper_->setFootholdsToReach(footholdsToReach);
    Pose result;
    EXPECT_TRUE(optimizationHelper_->optimize(result));
    sleepDurationBetweenTests_->sleep();
  }
}

TEST_F(PoseOptimizationSqpTest, FrontLegsHeightChange)
{
  Stance footholdsToReach({
    {LimbEnum::LF_LEG, Position(1.0, 0.2, 0.0)},
    {LimbEnum::RF_LEG, Position(1.0, -0.2, 0.0)},
    {LimbEnum::LH_LEG, Position(0.6, 0.2, 0.0)},
    {LimbEnum::RH_LEG, Position(0.6, -0.2, 0.0)} });

  for (double footPositionZ = -0.3; footPositionZ < 1.0; footPositionZ += 0.02) {
    footholdsToReach[LimbEnum::LF_LEG].z() = footPositionZ;
    footholdsToReach[LimbEnum::RF_LEG].z() = footPositionZ;
    optimizationHelper_->setFootholdsToReach(footholdsToReach);
    optimizationHelper_->setFootholdsInSupport(footholdsToReach, 0.05);
    Pose result;
    EXPECT_TRUE(optimizationHelper_->optimize(result));
    sleepDurationBetweenTests_->sleep();
  }
}

TEST_F(PoseOptimizationSqpTest, FrontLegsSlideOnStep)
{
  Stance footholdsToReach({
    {LimbEnum::LF_LEG, Position(1.0, 0.2, 0.3)},
    {LimbEnum::RF_LEG, Position(1.0, -0.2, 0.3)},
    {LimbEnum::LH_LEG, Position(0.6, 0.2, 0.0)},
    {LimbEnum::RH_LEG, Position(0.6, -0.2, 0.0)} });

  for (double footPositionX = 0.85; footPositionX < 1.5; footPositionX += 0.02) {
    footholdsToReach[LimbEnum::LF_LEG].x() = footPositionX;
    footholdsToReach[LimbEnum::RF_LEG].x() = footPositionX;
    optimizationHelper_->setFootholdsToReach(footholdsToReach);
    optimizationHelper_->setFootholdsInSupport(footholdsToReach, 0.05);
    Pose result;
    EXPECT_TRUE(optimizationHelper_->optimize(result));
    sleepDurationBetweenTests_->sleep();
  }
}

TEST_F(PoseOptimizationSqpTest, DISABLED_ThreeLegs)
{
  const Stance footPositions({
    {LimbEnum::LF_LEG, Position(0.3, 0.2, 0.0)},
    {LimbEnum::RF_LEG, Position(0.3, -0.2, 0.0)},
    {LimbEnum::LH_LEG, Position(-0.3, 0.2, 0.0)} });
  optimizationHelper_->setFootholdsToReach(footPositions);
  optimizationHelper_->setFootholdsInSupport(footPositions);

  Pose result;
  ASSERT_TRUE(optimizationHelper_->optimize(result));
//
//  Eigen::Vector3d expectedPosition(0.0, 0.0, 0.48);
//  RotationMatrix expectedOrientation; // Identity.
//
//  kindr::assertNear(expectedOrientation.matrix(), RotationMatrix(result.getRotation()).matrix(), 1e-2, KINDR_SOURCE_FILE_POS);
//  kindr::assertNear(expectedPosition, result.getPosition().vector(), 1e-3, KINDR_SOURCE_FILE_POS);
}

TEST_F(PoseOptimizationSqpTest, StairDescent)
{
  AdapterBase& adapter(*optimizationHelper_->getAdapterRosPtr());
  PoseConstraintsChecker::LimbLengths minLimbLenghts, maxLimbLenghts;

  minLimbLenghts[LimbEnum::LF_LEG] = 0.2;
  minLimbLenghts[LimbEnum::RF_LEG] = 0.2;
  minLimbLenghts[LimbEnum::LH_LEG] = 0.2;
  minLimbLenghts[LimbEnum::RH_LEG] = 0.2;

  maxLimbLenghts[LimbEnum::LF_LEG] = 0.575;
  maxLimbLenghts[LimbEnum::RF_LEG] = 0.575;
  maxLimbLenghts[LimbEnum::LH_LEG] = 0.575;
  maxLimbLenghts[LimbEnum::RH_LEG] = 0.595;

  Stance nominalStance({
    {LimbEnum::LF_LEG, Position( 0.33,  0.22, -0.5375)},
    {LimbEnum::RF_LEG, Position( 0.33, -0.22, -0.5375)},
    {LimbEnum::LH_LEG, Position(-0.33,  0.22, -0.5375)},
    {LimbEnum::RH_LEG, Position(-0.33, -0.22, -0.5375)} });

  Stance footholdsToReach({
    {LimbEnum::LF_LEG, Position(-1.20819, -0.171826, 0.552115)},
    {LimbEnum::RF_LEG, Position(-1.18284, 0.290253, 0.546179)},
    {LimbEnum::LH_LEG, Position(-0.882982, -0.183467, 0.388745)},
    {LimbEnum::RH_LEG, Position(-0.632574, 0.290462, 0.193111)} });

  Stance footholdsInSupport({
    {LimbEnum::LF_LEG, Position(-1.20819, -0.171826, 0.552115)},
    {LimbEnum::RF_LEG, Position(-1.18284, 0.290253, 0.546179)},
    {LimbEnum::LH_LEG, Position(-0.882982, -0.183467, 0.388745)} });

  optimizationHelper_->setNominalStanceInBaseFrame(nominalStance);
  optimizationHelper_->setFootholdsToReach(footholdsToReach);
  optimizationHelper_->setFootholdsInSupport(footholdsInSupport, 0.06);
  optimizationHelper_->setLimbLengthConstraints(minLimbLenghts, maxLimbLenghts);
  Pose result;
  EXPECT_TRUE(optimizationHelper_->optimize(result));
  sleepDurationBetweenTests_->sleep();
}
