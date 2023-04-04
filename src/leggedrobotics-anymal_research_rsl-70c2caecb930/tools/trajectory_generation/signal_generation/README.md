# Signal generation

Tool for generating a signal from various profiles.

Currently implemented profiles are a constant, a step, a ramp, a sine and a logarithmic chirp.
Profiles can be superposed arbitrarily.
Every profile has a range, defined by a start and an end.
Outside of this range, the profile is zero.

The primary type is templated and can be chosen as e.g. float or double.

**Author(s):** Christian Gehring, Remo Diethelm

## Generating signals

Example code generating a signal:

```
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
```

## Visualizing signals

Example code visualizing two signals:

```
// qt signal generation
#include "qt_signal_generation/qt_signal_generation.hpp"

using namespace signal_generation;
using namespace qt_signal_generation;

int main(int argc, char** argv)
{
  // Create first signal (float).
  SignalF signal1;
  signal1.addProfile(ProfileRampFPtr(new ProfileRampF(RangeF(1.0f, 4.0f), 1.0f)));

  // Create second signal (double).
  SignalD signal2;
  signal2.addProfile(ProfileConstantDPtr(new ProfileConstantD(RangeD(2.0, 5.0), 1.0)));
  signal2.addProfile(ProfileSineDPtr(new ProfileSineD(RangeD(2.0, 5.0), 1.0, 1.0, 0.0)));

  // Create a QT application (needs to be done before creating a signal visualizer).
  QApplication application(argc, argv);

  // Create a signal visualizer which uses a time step of 0.001 s for sampling.
  SignalVisualizer signalVisualizer(0.001);
  // Add the first signal in blue.
  signalVisualizer.addSignal(signal1, Qt::blue);
  // Add the second signal in red.
  signalVisualizer.addSignal(signal2, Qt::red);

  // Run the QT application which opens one window per signal visualizer.
  return application.exec();
}
```
