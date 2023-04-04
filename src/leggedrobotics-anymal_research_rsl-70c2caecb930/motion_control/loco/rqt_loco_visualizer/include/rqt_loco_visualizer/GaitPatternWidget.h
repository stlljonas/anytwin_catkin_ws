/******************************************************************************
 * Authors:                                                                   *
 *    C. Dario Bellicoso                                                      *
 *    Samuel Bachmann <sbachmann@anybotics.com>                               *
 ******************************************************************************/

#pragma once

#include <mutex>

#include <QPainter>
#include <QTimer>
#include <QVector>
#include <QWidget>

#include "rqt_loco_visualizer/GaitPatternContainer.h"

namespace rqt_loco_visualizer {

class GaitPatternWidget : public QWidget {
Q_OBJECT

public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit GaitPatternWidget(QWidget *parent = nullptr);

  ~GaitPatternWidget() final;

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  void setGaitContainer(std::vector<GaitPatternContainer>& gpContainer);

  void setStridePhase(double stridePhase);

private:

  void paintWhiteBar(
      QPainter& painter,
      double intervalStart, double intervalEnd,
      int gaitBarPositionY, int i, int j,
      int boxLength);

  /* ======================================================================== */
  /* Constants                                                                */
  /* ======================================================================== */

  const std::string TAG = "GaitPatternWidget";

  const int REPAINT_UPDATE_RATE = 33; // 30 Hz

  const int footLabelMarginLeft_ = 5;
  const int footLabelMarginRight_ = 10;

  const int gaitBarHeight_ = 20;
  const int gaitBarMarginLeft_ = 0;
  const int gaitBarMarginTopBottom_ = 10;
  const int gaitBarMarginRight_ = 5;

  const QVector<QString> footLabels_ = {"LF", "RF", "LH", "RH"};

  const QColor footLabelColor_ = Qt::black;

  const QVector<QColor> gaitBarColor_ = {
      QColor(214, 1, 39),
      QColor(231, 165, 0),
      QColor(0, 96, 40),
      QColor(49, 180, 48)
  };

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  QFont footLabelFont_;
  int footLabelWidth_ = 0;

  int footLabelPositionX_ = 0;
  int gaitBarPositionX_ = 0;
  int lineLength_ = 0;

  QTimer *repaintTimer_;

  std::mutex mutexGaitPatterns_;
  std::vector<GaitPatternContainer> gpContainer_;

  std::mutex mutexCursorPosition_;
  double cursorPosition_ = 0.0;

  int nrCycles_ = 1;

  double totalDuration_ = 1.0;

  /* ======================================================================== */
  /* Events                                                                   */
  /* ======================================================================== */

  void paintEvent(QPaintEvent *event) override;

};

} // namespace
