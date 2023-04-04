#pragma once


// qt
#include <QVBoxLayout>
#include <QWidget>

// qcustomplot
#include <qcustomplot/qcustomplot.h>

// signal generation
#include <signal_generation/signal_generation.hpp>


namespace qt_signal_generation {


class SignalVisualizerWidget : public QWidget
{
  Q_OBJECT

protected:
  const double rangeScaling_ = 1.2;
  const double timeStep_ = 0.0;

  QVBoxLayout* layout_ = nullptr;
  QCustomPlot* plotValue_ = nullptr;
  QCustomPlot* plotFirstDerivative_ = nullptr;
  QCustomPlot* plotSecondDerivative_ = nullptr;

  int currentGraph_ = 0;

public:
  SignalVisualizerWidget(const double timeStep, const int windowSizeX = 1024, const int windowSizeY = 768, QWidget* parent = nullptr);
  virtual ~SignalVisualizerWidget();

  template <typename PrimT_>
  void addSignal(const signal_generation::Signal<PrimT_>& signal, const QColor color)
  {
    const std::deque<std::pair<PrimT_, PrimT_>> timesAndValues =
        signal.sampleTimeAndValue(signal.getUnifiedRange(), timeStep_);
    const std::deque<std::pair<PrimT_, PrimT_>> timesAndFirstDerivatives =
        signal.sampleTimeAndFirstDerivative(signal.getUnifiedRange(), timeStep_);
    const std::deque<std::pair<PrimT_, PrimT_>> timesAndSecondDerivatives =
        signal.sampleTimeAndSecondDerivative(signal.getUnifiedRange(), timeStep_);

    plotValue_->addGraph();
    plotValue_->graph(currentGraph_)->setPen(QPen(color));
    for (const auto& timeAndValue : timesAndValues)
      plotValue_->graph(currentGraph_)->addData(timeAndValue.first, timeAndValue.second);
    plotValue_->rescaleAxes();
    plotValue_->xAxis->scaleRange(rangeScaling_);
    plotValue_->yAxis->scaleRange(rangeScaling_);

    plotFirstDerivative_->addGraph();
    plotFirstDerivative_->graph(currentGraph_)->setPen(QPen(color));
    for (const auto& timeAndFirstDerivative : timesAndFirstDerivatives)
      plotFirstDerivative_->graph(currentGraph_)->addData(timeAndFirstDerivative.first, timeAndFirstDerivative.second);
    plotFirstDerivative_->rescaleAxes();
    plotFirstDerivative_->xAxis->scaleRange(rangeScaling_);
    plotFirstDerivative_->yAxis->scaleRange(rangeScaling_);

    plotSecondDerivative_->addGraph();
    plotSecondDerivative_->graph(currentGraph_)->setPen(QPen(color));
    for (const auto& timeAndSecondDerivative : timesAndSecondDerivatives)
      plotSecondDerivative_->graph(currentGraph_)->addData(timeAndSecondDerivative.first, timeAndSecondDerivative.second);
    plotSecondDerivative_->rescaleAxes();
    plotSecondDerivative_->xAxis->scaleRange(rangeScaling_);
    plotSecondDerivative_->yAxis->scaleRange(rangeScaling_);

    currentGraph_++;
  }
};


} // qt_signal_generation
