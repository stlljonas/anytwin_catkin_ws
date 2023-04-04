/*
 * StairsGeometryTest.cpp
 *
 *  Created on: Nov 28, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_stair_climbing/StairsGeometry.hpp"

// gtest
#include <gtest/gtest.h>

using namespace free_gait_stair_climbing;

TEST(StairsGeometry, allGeneral)
{
  StairsGeometry stairsGeometry;
  stairsGeometry.setFrameId("stairs");
  stairsGeometry.setGeneralStepGeometry(0.1, 0.2);
  stairsGeometry.setNumberOfSteps(10);

  // General info.
  EXPECT_EQ("stairs", stairsGeometry.getFrameId());
  EXPECT_EQ(10.0, stairsGeometry.getNumberOfSteps());

  // Rise & run.
  for (size_t i = 1; i <= 10; ++i) {
    EXPECT_EQ(0.1, stairsGeometry.getRise(i));
    EXPECT_EQ(0.2, stairsGeometry.getRun(i));
  }

  // Positions.
  EXPECT_DOUBLE_EQ(0.1, stairsGeometry.getCenterPosition(1).x());
  EXPECT_DOUBLE_EQ(0.0, stairsGeometry.getCenterPosition(1).y());
  EXPECT_DOUBLE_EQ(0.1, stairsGeometry.getCenterPosition(1).z());
  EXPECT_DOUBLE_EQ(0.3, stairsGeometry.getCenterPosition(2).x());
  EXPECT_DOUBLE_EQ(0.2, stairsGeometry.getCenterPosition(2).z());
  EXPECT_DOUBLE_EQ(1.9, stairsGeometry.getCenterPosition(10).x());
  EXPECT_DOUBLE_EQ(1.0, stairsGeometry.getCenterPosition(10).z());
}

TEST(StairsGeometry, specialFirstStep)
{
  StairsGeometry stairsGeometry;
  stairsGeometry.setFrameId("stairs");
  stairsGeometry.setGeneralStepGeometry(0.1, 0.2);
  stairsGeometry.setFirstStepGeometry(0.15, 0.25);
  stairsGeometry.setNumberOfSteps(10);

  // General info.
  EXPECT_EQ("stairs", stairsGeometry.getFrameId());
  EXPECT_EQ(10.0, stairsGeometry.getNumberOfSteps());

  // Rise & run.
  EXPECT_EQ(0.15, stairsGeometry.getRise(1));
  EXPECT_EQ(0.25, stairsGeometry.getRun(1));
  for (size_t i = 2; i <= 10; ++i) {
    EXPECT_EQ(0.1, stairsGeometry.getRise(i));
    EXPECT_EQ(0.2, stairsGeometry.getRun(i));
  }

  // Positions.
  EXPECT_DOUBLE_EQ(0.125, stairsGeometry.getCenterPosition(1).x());
  EXPECT_DOUBLE_EQ(0.0, stairsGeometry.getCenterPosition(1).y());
  EXPECT_DOUBLE_EQ(0.15, stairsGeometry.getCenterPosition(1).z());
  EXPECT_DOUBLE_EQ(0.35, stairsGeometry.getCenterPosition(2).x());
  EXPECT_DOUBLE_EQ(0.25, stairsGeometry.getCenterPosition(2).z());
  EXPECT_DOUBLE_EQ(1.95, stairsGeometry.getCenterPosition(10).x());
  EXPECT_DOUBLE_EQ(1.05, stairsGeometry.getCenterPosition(10).z());
}

TEST(StairsGeometry, copy)
{
  StairsGeometry stairsGeometry;
  stairsGeometry.setFrameId("stairs");
  stairsGeometry.setGeneralStepGeometry(0.1, 0.2);
  stairsGeometry.setFirstStepGeometry(0.15, 0.25);
  stairsGeometry.setNumberOfSteps(10);

  {
    StairsGeometry copyOfStairsGeometry(stairsGeometry);
    EXPECT_DOUBLE_EQ(stairsGeometry.getCenterPosition(1).x(), copyOfStairsGeometry.getCenterPosition(1).x());
  }
}
