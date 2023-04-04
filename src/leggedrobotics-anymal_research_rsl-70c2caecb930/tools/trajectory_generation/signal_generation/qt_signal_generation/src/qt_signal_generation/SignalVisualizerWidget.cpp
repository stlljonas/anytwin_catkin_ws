// qt signal generation
#include "qt_signal_generation/SignalVisualizerWidget.hpp"


namespace qt_signal_generation {


SignalVisualizerWidget::SignalVisualizerWidget(const double timeStep, const int windowSizeX, const int windowSizeY, QWidget* parent)
: QWidget(parent),
  timeStep_(timeStep)
{
  resize(windowSizeX, windowSizeY);
  show();

  layout_ = new QVBoxLayout();
  setLayout(layout_);

  plotValue_ = new QCustomPlot();
  plotValue_->xAxis->setLabel(QString::fromStdString("Time [s]"));
  plotValue_->yAxis->setLabel(QString::fromStdString("Value [-]"));
  plotValue_->axisRect()->setupFullAxesBox();
  connect(plotValue_->xAxis, SIGNAL(rangeChanged(QCPRange)), plotValue_->xAxis2, SLOT(setRange(QCPRange)));
  connect(plotValue_->yAxis, SIGNAL(rangeChanged(QCPRange)), plotValue_->yAxis2, SLOT(setRange(QCPRange)));
  plotValue_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  layout_->addWidget(plotValue_);

  plotFirstDerivative_ = new QCustomPlot();
  plotFirstDerivative_->xAxis->setLabel(QString::fromStdString("Time [s]"));
  plotFirstDerivative_->yAxis->setLabel(QString::fromStdString("First derivative [-]"));
  plotFirstDerivative_->axisRect()->setupFullAxesBox();
  connect(plotFirstDerivative_->xAxis, SIGNAL(rangeChanged(QCPRange)), plotFirstDerivative_->xAxis2, SLOT(setRange(QCPRange)));
  connect(plotFirstDerivative_->yAxis, SIGNAL(rangeChanged(QCPRange)), plotFirstDerivative_->yAxis2, SLOT(setRange(QCPRange)));
  plotFirstDerivative_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  layout_->addWidget(plotFirstDerivative_);

  plotSecondDerivative_ = new QCustomPlot();
  plotSecondDerivative_->xAxis->setLabel(QString::fromStdString("Time [s]"));
  plotSecondDerivative_->yAxis->setLabel(QString::fromStdString("Second derivative [-]"));
  plotSecondDerivative_->axisRect()->setupFullAxesBox();
  connect(plotSecondDerivative_->xAxis, SIGNAL(rangeChanged(QCPRange)), plotSecondDerivative_->xAxis2, SLOT(setRange(QCPRange)));
  connect(plotSecondDerivative_->yAxis, SIGNAL(rangeChanged(QCPRange)), plotSecondDerivative_->yAxis2, SLOT(setRange(QCPRange)));
  plotSecondDerivative_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  layout_->addWidget(plotSecondDerivative_);
}

SignalVisualizerWidget::~SignalVisualizerWidget() {}


} // qt_signal_generation
