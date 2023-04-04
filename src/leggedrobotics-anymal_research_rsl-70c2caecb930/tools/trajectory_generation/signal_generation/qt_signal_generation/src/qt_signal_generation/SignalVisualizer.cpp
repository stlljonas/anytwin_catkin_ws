// qt signal generation
#include "qt_signal_generation/SignalVisualizer.hpp"


namespace qt_signal_generation {


SignalVisualizer::SignalVisualizer(const double timeStep, const int windowSizeX, const int windowSizeY)
: signalVisualizerWidget_(new SignalVisualizerWidget(timeStep, windowSizeX, windowSizeY)) {}

SignalVisualizer::~SignalVisualizer() {}


} // qt_signal_generation
