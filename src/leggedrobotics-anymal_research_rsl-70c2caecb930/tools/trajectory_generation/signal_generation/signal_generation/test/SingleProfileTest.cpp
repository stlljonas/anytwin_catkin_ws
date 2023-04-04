// gtest
#include <gtest/gtest.h>

// signal generation
#include "signal_generation/signal_generation.hpp"


using namespace signal_generation;


TEST(SingleProfile, profileConstant)
{
  SignalD signal;
  signal.addProfile(ProfileConstantDPtr(new ProfileConstantD(RangeD(4.0, 6.0), 1.0)));

  EXPECT_DOUBLE_EQ(signal.getValue(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getValue(4.0),   1.0);
  EXPECT_DOUBLE_EQ(signal.getValue(5.0),   1.0);
  EXPECT_DOUBLE_EQ(signal.getValue(5.999), 1.0);
  EXPECT_DOUBLE_EQ(signal.getValue(6.0),   0.0);

  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(4.0),   0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(5.0),   0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(5.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(6.0),   0.0);

  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(4.0),   0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(5.0),   0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(5.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(6.0),   0.0);
}


TEST(SingleProfile, profileStep)
{
  SignalD signal;
  signal.addProfile(ProfileStepDPtr(new ProfileStepD(RangeD(4.0, 6.0), 1.0, 1.0)));

  EXPECT_DOUBLE_EQ(signal.getValue(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getValue(4.0),   0.0);
  EXPECT_DOUBLE_EQ(signal.getValue(5.0),   1.0);
  EXPECT_DOUBLE_EQ(signal.getValue(5.999), 1.0);
  EXPECT_DOUBLE_EQ(signal.getValue(6.0),   0.0);

  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(4.0),   0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(5.0),   std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(5.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(6.0),   0.0);

  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(4.0),   0.0);
  EXPECT_TRUE(std::isnan(signal.getSecondDerivative(5.0)));
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(5.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(6.0),   0.0);
}


TEST(SingleProfile, profileRamp)
{
  SignalD signal;
  signal.addProfile(ProfileRampDPtr(new ProfileRampD(RangeD(4.0, 6.0), 1.0)));

  EXPECT_DOUBLE_EQ(signal.getValue(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getValue(4.0),   0.0);
  EXPECT_DOUBLE_EQ(signal.getValue(5.0),   0.5);
  EXPECT_DOUBLE_EQ(signal.getValue(5.999), 1.999/2.0);
  EXPECT_DOUBLE_EQ(signal.getValue(6.0),   0.0);

  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(4.0),   0.5);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(5.0),   0.5);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(5.999), 0.5);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(6.0),   0.0);

  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(4.0),   0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(5.0),   0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(5.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(6.0),   0.0);
}


TEST(SingleProfile, profileSine)
{
  SignalD signal;
  signal.addProfile(ProfileSineDPtr(new ProfileSineD(RangeD(4.0, 6.0), 1.0, 1.0/(2.0*M_PI), 0.0)));

  EXPECT_DOUBLE_EQ(signal.getValue(3.999), 0.0);
  EXPECT_NEAR(signal.getValue(4.0),   std::sin(0.0),   1e-9);
  EXPECT_NEAR(signal.getValue(4.5),   std::sin(0.5),   1e-9);
  EXPECT_NEAR(signal.getValue(5.0),   std::sin(1.0),   1e-9);
  EXPECT_NEAR(signal.getValue(5.5),   std::sin(1.5),   1e-9);
  EXPECT_NEAR(signal.getValue(5.999), std::sin(1.999), 1e-9);
  EXPECT_DOUBLE_EQ(signal.getValue(6.0),   0.0);

  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(3.999), 0.0);
  EXPECT_NEAR(signal.getFirstDerivative(4.0),   std::cos(0.0),   1e-9);
  EXPECT_NEAR(signal.getFirstDerivative(4.5),   std::cos(0.5),   1e-9);
  EXPECT_NEAR(signal.getFirstDerivative(5.0),   std::cos(1.0),   1e-9);
  EXPECT_NEAR(signal.getFirstDerivative(5.5),   std::cos(1.5),   1e-9);
  EXPECT_NEAR(signal.getFirstDerivative(5.999), std::cos(1.999), 1e-9);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(6.0),   0.0);

  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(3.999), 0.0);
  EXPECT_NEAR(signal.getSecondDerivative(4.0),   -std::sin(0.0),   1e-9);
  EXPECT_NEAR(signal.getSecondDerivative(4.5),   -std::sin(0.5),   1e-9);
  EXPECT_NEAR(signal.getSecondDerivative(5.0),   -std::sin(1.0),   1e-9);
  EXPECT_NEAR(signal.getSecondDerivative(5.5),   -std::sin(1.5),   1e-9);
  EXPECT_NEAR(signal.getSecondDerivative(5.999), -std::sin(1.999), 1e-9);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(6.0),   0.0);
}


TEST(SingleProfile, profileLogChirpUpSweep)
{
  SignalD signal;
  signal.addProfile(ProfileLogChirpUpSweepDPtr(new ProfileLogChirpUpSweepD(RangeD(4.0, 6.0), 1.0, 1.0, 10.0)));

  EXPECT_DOUBLE_EQ(signal.getValue(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getValue(4.0),   0.0);
  EXPECT_DOUBLE_EQ(signal.getValue(5.0),   -0.69306239547363824);
  EXPECT_DOUBLE_EQ(signal.getValue(5.999), -0.93587325104142771);
  EXPECT_DOUBLE_EQ(signal.getValue(6.0),   0.0);

  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(4.0),   6.2831853071795862);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(5.0),   14.323244316857208);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(5.999), 22.112504757442604);
  EXPECT_DOUBLE_EQ(signal.getFirstDerivative(6.0),   0.0);

  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(3.999), 0.0);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(4.0),   7.2337844124154662);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(5.0),   290.10031116749968);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(5.999), 3711.6399377948615);
  EXPECT_DOUBLE_EQ(signal.getSecondDerivative(6.0),   0.0);
}
