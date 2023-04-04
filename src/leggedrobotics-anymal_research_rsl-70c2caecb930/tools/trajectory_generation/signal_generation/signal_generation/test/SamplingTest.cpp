// gtest
#include <gtest/gtest.h>

// signal generation
#include "signal_generation/signal_generation.hpp"


using namespace signal_generation;


TEST(Sampling, sampling)
{
  SignalD signal;
  signal.addProfile(ProfileRampDPtr(new ProfileRampD(RangeD(4.0, 6.0), 1.0)));
  signal.addProfile(ProfileSineDPtr(new ProfileSineD(RangeD(3.5, 5.5), 1.0, 1.0/(2.0*M_PI), 0.0)));

  std::deque<double> values = signal.sampleValue(RangeD(3.0, 5.0), 0.1);
  EXPECT_DOUBLE_EQ(values.size(), 21);
  EXPECT_DOUBLE_EQ(values[0], 0.0);
  EXPECT_DOUBLE_EQ(values[1], 0.0);
  EXPECT_NEAR(values[ 5], 0.00 + std::sin(0.0), 1e-9);
  EXPECT_NEAR(values[ 6], 0.00 + std::sin(0.1), 1e-9);
  EXPECT_NEAR(values[10], 0.00 + std::sin(0.5), 1e-9);
  EXPECT_NEAR(values[11], 0.05 + std::sin(0.6), 1e-9);
  EXPECT_NEAR(values[12], 0.10 + std::sin(0.7), 1e-9);
  EXPECT_NEAR(values[15], 0.25 + std::sin(1.0), 1e-9);
  EXPECT_NEAR(values[20], 0.50 + std::sin(1.5), 1e-9);

  std::deque<double> firstDerivatives = signal.sampleFirstDerivative(RangeD(3.0, 5.0), 0.1);
  EXPECT_DOUBLE_EQ(firstDerivatives.size(), 21);
  EXPECT_DOUBLE_EQ(firstDerivatives[0], 0.0);
  EXPECT_DOUBLE_EQ(firstDerivatives[1], 0.0);
  EXPECT_NEAR(firstDerivatives[ 5], 0.0 + std::cos(0.0), 1e-9);
  EXPECT_NEAR(firstDerivatives[ 6], 0.0 + std::cos(0.1), 1e-9);
  EXPECT_NEAR(firstDerivatives[10], 0.5 + std::cos(0.5), 1e-9);
  EXPECT_NEAR(firstDerivatives[11], 0.5 + std::cos(0.6), 1e-9);
  EXPECT_NEAR(firstDerivatives[12], 0.5 + std::cos(0.7), 1e-9);
  EXPECT_NEAR(firstDerivatives[15], 0.5 + std::cos(1.0), 1e-9);
  EXPECT_NEAR(firstDerivatives[20], 0.5 + std::cos(1.5), 1e-9);

  std::deque<double> secondDerivatives = signal.sampleSecondDerivative(RangeD(3.0, 5.0), 0.1);
  EXPECT_DOUBLE_EQ(secondDerivatives.size(), 21);
  EXPECT_DOUBLE_EQ(secondDerivatives[0], 0.0);
  EXPECT_DOUBLE_EQ(secondDerivatives[1], 0.0);
  EXPECT_NEAR(secondDerivatives[ 5], -std::sin(0.0), 1e-9);
  EXPECT_NEAR(secondDerivatives[ 6], -std::sin(0.1), 1e-9);
  EXPECT_NEAR(secondDerivatives[10], -std::sin(0.5), 1e-9);
  EXPECT_NEAR(secondDerivatives[11], -std::sin(0.6), 1e-9);
  EXPECT_NEAR(secondDerivatives[12], -std::sin(0.7), 1e-9);
  EXPECT_NEAR(secondDerivatives[15], -std::sin(1.0), 1e-9);
  EXPECT_NEAR(secondDerivatives[20], -std::sin(1.5), 1e-9);
}
