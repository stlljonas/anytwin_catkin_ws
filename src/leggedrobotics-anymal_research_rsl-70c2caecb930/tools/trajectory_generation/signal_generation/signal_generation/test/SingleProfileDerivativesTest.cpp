// gtest
#include <gtest/gtest.h>

// signal generation
#include "signal_generation/signal_generation.hpp"


using namespace signal_generation;


#define CHECK_FIRST_DERIVATIVE(signal, x, dt, tol) \
{ \
  EXPECT_NEAR((signal.getValue(x+dt) - signal.getValue(x))/dt, signal.getFirstDerivative(x), tol); \
}

#define CHECK_SECOND_DERIVATIVE(signal, x, dt, tol) \
{ \
  EXPECT_NEAR((signal.getFirstDerivative(x+dt) - signal.getFirstDerivative(x))/dt, signal.getSecondDerivative(x), tol); \
}


TEST(SingleProfileDerivatives, profileConstant)
{
  const double dt = 1e-6;
  const double tol = 1e-6;

  SignalD signal;
  signal.addProfile(ProfileConstantDPtr(new ProfileConstantD(RangeD(4.0, 6.0), 1.0)));

  CHECK_FIRST_DERIVATIVE(signal, 4.50, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 4.75, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.00, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.25, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.50, dt, tol);

  CHECK_SECOND_DERIVATIVE(signal, 4.50, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 4.75, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.00, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.25, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.50, dt, tol);
}


TEST(SingleProfileDerivatives, profileStep)
{
  const double dt = 1e-6;
  const double tol = 1e-6;

  SignalD signal;
  signal.addProfile(ProfileStepDPtr(new ProfileStepD(RangeD(4.0, 6.0), 1.0, 1.0)));

  CHECK_FIRST_DERIVATIVE(signal, 4.50, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 4.75, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.25, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.50, dt, tol);

  CHECK_SECOND_DERIVATIVE(signal, 4.50, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 4.75, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.25, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.50, dt, tol);
}


TEST(SingleProfileDerivatives, profileRamp)
{
  const double dt = 1e-6;
  const double tol = 1e-6;

  SignalD signal;
  signal.addProfile(ProfileRampDPtr(new ProfileRampD(RangeD(4.0, 6.0), 1.0)));

  CHECK_FIRST_DERIVATIVE(signal, 4.50, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 4.75, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.00, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.25, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.50, dt, tol);

  CHECK_SECOND_DERIVATIVE(signal, 4.50, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 4.75, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.00, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.25, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.50, dt, tol);
}


TEST(SingleProfileDerivatives, profileSine)
{
  const double dt = 1e-6;
  const double tol = 1e-6;

  SignalD signal;
  signal.addProfile(ProfileSineDPtr(new ProfileSineD(RangeD(4.0, 6.0), 1.0, 1.0/(2.0*M_PI), 0.0)));

  CHECK_FIRST_DERIVATIVE(signal, 4.50, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 4.75, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.00, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.25, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.50, dt, tol);

  CHECK_SECOND_DERIVATIVE(signal, 4.50, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 4.75, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.00, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.25, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.50, dt, tol);
}


TEST(SingleProfileDerivatives, profileLogChirpUpSweep)
{
  const double dt = 1e-9;
  const double tol = 1e-4;

  SignalD signal;
  signal.addProfile(ProfileLogChirpUpSweepDPtr(new ProfileLogChirpUpSweepD(RangeD(4.0, 6.0), 1.0, 1.0, 10.0)));

  CHECK_FIRST_DERIVATIVE(signal, 4.50, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 4.75, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.00, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.25, dt, tol);
  CHECK_FIRST_DERIVATIVE(signal, 5.50, dt, tol);

  CHECK_SECOND_DERIVATIVE(signal, 4.50, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 4.75, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.00, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.25, dt, tol);
  CHECK_SECOND_DERIVATIVE(signal, 5.50, dt, tol);
}
