// gtest
#include <gtest/gtest.h>

// signal generation
#include "signal_generation/signal_generation.hpp"


using namespace signal_generation;


TEST(Range, emptyRange)
{
  RangeD range1(1.0, 2.0);
  RangeD range2(2.0, 2.0);
  RangeD range3(3.0, 2.0);

  EXPECT_FALSE(range1.isEmpty());
  EXPECT_FALSE(range2.isEmpty());
  EXPECT_TRUE(range3.isEmpty());
}


TEST(Range, combinedRange)
{
  SignalD signal;
  signal.addProfile(ProfileConstantDPtr(new ProfileConstantD(RangeD(-5.0, 1.0), 1.0)));
  signal.addProfile(ProfileStepDPtr(new ProfileStepD(RangeD(-6.0, 3.0), 5.0, 1.0)));
  signal.addProfile(ProfileRampDPtr(new ProfileRampD(RangeD(-2.0, 3.0), 1.0)));
  signal.addProfile(ProfileSineDPtr(new ProfileSineD(RangeD(4.0, 5.0), 1.0, 2.0*M_PI, 0.0)));

  EXPECT_DOUBLE_EQ(signal.getUnifiedRange().getStart(), -6.0);
  EXPECT_DOUBLE_EQ(signal.getUnifiedRange().getEnd(), 5.0);
  EXPECT_FALSE(signal.getUnifiedRange().isEmpty());

  EXPECT_DOUBLE_EQ(signal.getIntersectedRange().getStart(), 4.0);
  EXPECT_DOUBLE_EQ(signal.getIntersectedRange().getEnd(), 1.0);
  EXPECT_TRUE(signal.getIntersectedRange().isEmpty());
}
