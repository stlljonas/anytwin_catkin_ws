// STL
#include <deque>

// signal generation
#include <signal_generation/signal_generation.hpp>

using namespace signal_generation;

int main(int argc, char** argv)
{
  // Create an empty signal.
  SignalD signal;

  // Add a constant profile at [-5.0, 1.0) with height 1.0.
  signal.addProfile(ProfileConstantDPtr(new ProfileConstantD(RangeD(-5.0, 1.0), 1.0)));
  // Add a step profile at [-6.0, 3.0) at relative position 7.0 with height 2.0.
  signal.addProfile(ProfileStepDPtr(new ProfileStepD(RangeD(-6.0, 3.0), 7.0, 2.0)));
  // Add a ramp profile at [-2.0, 3.0) with end height 1.0 (start height is always 0.0).
  signal.addProfile(ProfileRampDPtr(new ProfileRampD(RangeD(-2.0, 3.0), 1.0)));
  // Add a sine profile at [4.0, 5.0) with amplitude 1.0, frequency 1.0/(2.0*PI) and phase shift 0.0.
  signal.addProfile(ProfileSineDPtr(new ProfileSineD(RangeD(4.0, 5.0), 1.0, 1.0/(2.0*M_PI), 0.0)));
  // Add a logarithmic chirp profile at [4.0, 6.0) with amplitude 1.0, start frequency 1.0 and end frequency 10.0.
  signal.addProfile(ProfileLogChirpUpSweepDPtr(new ProfileLogChirpUpSweepD(RangeD(4.0, 6.0), 1.0, 1.0, 10.0)));

  // Get the value of the signal at position 1.0.
  double value = signal.getValue(1.0);
  // Get the first derivative of the signal at position 2.0.
  double firstDerivative = signal.getFirstDerivative(2.0);
  // Get the second derivative of the signal at position 3.0.
  double secondDerivative = signal.getSecondDerivative(3.0);

  // Get the unified range of all profiles within the signal.
  RangeD unifiedRange = signal.getUnifiedRange();
  // Get the intersected range of all profiles within the signal.
  RangeD intersectedRange = signal.getIntersectedRange();

  // Sample the value in the range [1.0, 3.0) with a time step of 0.1s.
  std::deque<double> values = signal.sampleValue(RangeD(1.0, 3.0), 0.1);
  // Sample the first derivative in the range [1.0, 3.0) with a time step of 0.1s.
  std::deque<double> firstDerivatives = signal.sampleFirstDerivative(RangeD(1.0, 3.0), 0.1);
  // Sample the second derivative in the range [1.0, 3.0) with a time step of 0.1s.
  std::deque<double> secondDerivatives = signal.sampleSecondDerivative(RangeD(1.0, 3.0), 0.1);

  // Sample the time and the value in the range [1.0, 3.0) with a time step of 0.1s.
  std::deque<std::pair<double, double>> timesAndvalues = signal.sampleTimeAndValue(RangeD(1.0, 3.0), 0.1);
  // Sample the time and the first derivative in the range [1.0, 3.0) with a time step of 0.1s.
  std::deque<std::pair<double, double>> timesAndFirstDerivatives = signal.sampleTimeAndFirstDerivative(RangeD(1.0, 3.0), 0.1);
  // Sample the time and the second derivative in the range [1.0, 3.0) with a time step of 0.1s.
  std::deque<std::pair<double, double>> timesAndSecondDerivatives = signal.sampleTimeAndSecondDerivative(RangeD(1.0, 3.0), 0.1);
}
