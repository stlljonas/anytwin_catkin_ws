// gtest
#include <gtest/gtest.h>

// signal generation
#include "signal_generation/signal_generation.hpp"


using namespace signal_generation;


TEST(CombinedProfiles, value)
{
  SignalD signal;
  signal.addProfile(ProfileConstantDPtr(new ProfileConstantD(RangeD(-5.0, 1.0), 1.0)));
  signal.addProfile(ProfileStepDPtr(new ProfileStepD(RangeD(-6.0, 3.0), 7.0, 1.0)));
  signal.addProfile(ProfileRampDPtr(new ProfileRampD(RangeD(-2.0, 3.0), 1.0)));
  signal.addProfile(ProfileSineDPtr(new ProfileSineD(RangeD(2.0, 5.0), 1.0, 1.0/(2.0*M_PI), 0.0)));

  EXPECT_DOUBLE_EQ(signal.getValue(-7.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getValue(-6.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getValue(-5.0), 1.0);
  EXPECT_DOUBLE_EQ(signal.getValue(-4.0), 1.0);
  EXPECT_DOUBLE_EQ(signal.getValue(-3.0), 1.0);
  EXPECT_DOUBLE_EQ(signal.getValue(-2.0), 1.0 + 0.0*1.0/5.0);
  EXPECT_DOUBLE_EQ(signal.getValue(-1.0), 1.0 + 1.0*1.0/5.0);
  EXPECT_DOUBLE_EQ(signal.getValue( 0.0), 1.0 + 2.0*1.0/5.0);
  EXPECT_DOUBLE_EQ(signal.getValue( 1.0), 3.0*1.0/5.0 + 1.0);
  EXPECT_DOUBLE_EQ(signal.getValue( 2.0), 4.0*1.0/5.0 + std::sin(0.0) + 1.0);
  EXPECT_DOUBLE_EQ(signal.getValue( 3.0), std::sin(1.0));
  EXPECT_DOUBLE_EQ(signal.getValue( 4.0), std::sin(2.0));
  EXPECT_DOUBLE_EQ(signal.getValue( 5.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getValue( 6.0), 0.0);

  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(-7.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(-6.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(-5.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(-4.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(-3.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(-2.0), 1.0/5.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(-1.0), 1.0/5.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative( 0.0), 1.0/5.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative( 1.0), 1.0/5.0 + std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative( 2.0), 1.0/5.0 + std::cos(0.0));
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative( 3.0), std::cos(1.0));
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative( 4.0), std::cos(2.0));
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative( 5.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative( 6.0), 0.0);

  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(-7.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(-6.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(-5.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(-4.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(-3.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(-2.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative( 0.0), 0.0);
  EXPECT_TRUE(std::isnan(signal.getSecondDerivative(1.0)));
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative( 2.0), -std::sin(0.0));
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative( 3.0), -std::sin(1.0));
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative( 4.0), -std::sin(2.0));
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative( 5.0), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative( 6.0), 0.0);
}
