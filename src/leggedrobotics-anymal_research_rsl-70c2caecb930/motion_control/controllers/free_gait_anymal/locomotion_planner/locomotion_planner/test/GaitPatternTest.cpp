/*
 * GaitPatternTest.cpp
 *
 *  Created on: Mar 12, 2017
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich
 */

#include "locomotion_planner/common/type_defs.hpp"
#include "locomotion_planner/gait_pattern/GaitPatternCrawling.hpp"

// gtest
#include <gtest/gtest.h>

using namespace locomotion_planner;

TEST(GaitPatternCrawling, Iteration)
{
  GaitPatternCrawling gaitPattern;
  gaitPattern.setDirection(GaitPatternBase::Direction::FORWARD);
  gaitPattern.setFirstStep(LimbEnum::LH_LEG);
  EXPECT_EQ(LimbEnum::LH_LEG, gaitPattern.getCurrentLimb());
  gaitPattern.advance();
  EXPECT_EQ(LimbEnum::LF_LEG, gaitPattern.getCurrentLimb());
  gaitPattern.advance();
  EXPECT_EQ(LimbEnum::RH_LEG, gaitPattern.getCurrentLimb());
}
