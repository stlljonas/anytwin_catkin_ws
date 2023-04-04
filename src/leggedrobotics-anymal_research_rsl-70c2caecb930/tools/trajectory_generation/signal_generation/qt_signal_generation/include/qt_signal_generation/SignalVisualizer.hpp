#pragma once


// qt signal generation
#include "qt_signal_generation/SignalVisualizerWidget.hpp"


namespace qt_signal_generation {


class SignalVisualizer
{
protected:
  SignalVisualizerWidget* signalVisualizerWidget_ = nullptr;

public:
  SignalVisualizer(const double timeStep, const int windowSizeX = 1024, const int windowSizeY = 768);
  virtual ~SignalVisualizer();

  template <typename PrimT_>
  void addSignal(const signal_generation::Signal<PrimT_>& signal, const QColor color)
  {
    signalVisualizerWidget_->addSignal(signal, color);
  }
};


} // qt_signal_generation
