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
