/******************************************************************************
 * Authors:                                                                   *
 *    C. Dario Bellicoso                                                      *
 *    Samuel Bachmann <sbachmann@anybotics.com>                               *
 ******************************************************************************/

#include <ros/ros.h>

#include "rqt_loco_visualizer/GaitPatternWidget.h"

namespace rqt_loco_visualizer {

/* ========================================================================== */
/* Inline Functions                                                           */
/* ========================================================================== */

inline void boundToRange(double *v, double min, double max) {
  if (*v < min) *v = min;
  if (*v > max) *v = max;
}

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

GaitPatternWidget::GaitPatternWidget(QWidget *parent) :
    QWidget(parent) {
  setObjectName("GaitPatternWidget");

  // Initialize foot label font. Get max foot label width.
  footLabelFont_.setFamily("Helvetica");
  footLabelFont_.setPixelSize(20);
  QFontMetrics footLabelFontMetrics(footLabelFont_);
  footLabelWidth_ = 0;
  for (const auto &item : footLabels_) {
    int width = footLabelFontMetrics.width(item);
    if (width > footLabelWidth_) {
      footLabelWidth_ = width;
    }
  }

  // Initialize widget size independent variables.
  footLabelPositionX_ = footLabelMarginLeft_;
  gaitBarPositionX_ = footLabelPositionX_ + footLabelWidth_ +
      footLabelMarginRight_ + gaitBarMarginLeft_;
  lineLength_ = 5 * gaitBarMarginTopBottom_ + 4 * gaitBarHeight_;

  // Initialize repaint timer.
  repaintTimer_ = new QTimer(this);
  connect(repaintTimer_, SIGNAL(timeout()),
          this, SLOT(repaint()));
  repaintTimer_->start(REPAINT_UPDATE_RATE);

  // Initialize example gait pattern.
  gpContainer_.emplace_back(GaitPatternContainer());
  gpContainer_.emplace_back(GaitPatternContainer());

  nrCycles_ = (int)gpContainer_.size();

  totalDuration_ = 0.0;
  for (auto &container : gpContainer_) {
    container.setStartPhase(0.0);
    container.setDuration(1.0);
    totalDuration_ += container.getDuration();

    // lf
    container.setLiftOffPhase(0, 0);
    container.setTouchDownPhase(0.25, 0);
    container.setShouldBeGrounded(true, 0);
    // rf
    container.setLiftOffPhase(0.25, 1);
    container.setTouchDownPhase(0.5, 1);
    container.setShouldBeGrounded(true, 1);
    // lh
    container.setLiftOffPhase(0.5, 2);
    container.setTouchDownPhase(0.75, 2);
    container.setShouldBeGrounded(true, 2);
    // rh
    container.setLiftOffPhase(0.75, 3);
    container.setTouchDownPhase(1, 3);
    container.setShouldBeGrounded(true, 3);

    container.setStridePhase(0.0);
  }
}

GaitPatternWidget::~GaitPatternWidget() {
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void GaitPatternWidget::setGaitContainer(
    std::vector<GaitPatternContainer> &gpContainer) {
  std::lock_guard<std::mutex> lock(mutexGaitPatterns_);
  gpContainer_ = gpContainer;

  totalDuration_ = 0.0;
  for (const auto &container : gpContainer_) {
    totalDuration_ += container.getDuration();
  }

  if (totalDuration_ < 1e-4) {
    totalDuration_ = 1.0;
  }

  double phase = 0.0;
  for (auto &container : gpContainer_) {
    container.setStartPhase(phase);
    phase += container.getDuration() / totalDuration_;
  }
}

void GaitPatternWidget::setStridePhase(double stridePhase) {
  std::lock_guard<std::mutex> lock(mutexCursorPosition_);
  cursorPosition_ = stridePhase;
}

/* ========================================================================== */
/* Events                                                                     */
/* ========================================================================== */

void GaitPatternWidget::paintEvent(QPaintEvent *event) {
//  ros::Time start = ros::Time::now();

  // Variables.
  int gaitBarWidth = 0;
  int boxLength = 0;

  // Initialize painter.
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  {
    std::lock_guard<std::mutex> lock(mutexGaitPatterns_);

    nrCycles_ = (int)gpContainer_.size();

    // Check for division by zero.
    if (nrCycles_ == 0) return;
    if (totalDuration_ == 0.0) return;

    // Draw foot labels.
    painter.setPen(footLabelColor_);
    painter.setFont(footLabelFont_);
    int footLabelPositionY = gaitBarMarginTopBottom_ + gaitBarHeight_;
    for (const auto &item : footLabels_) {
      painter.drawText(footLabelPositionX_, footLabelPositionY, item);
      footLabelPositionY += gaitBarMarginTopBottom_ + gaitBarHeight_;
    }

    // Draw gait bars.
    gaitBarWidth = (this->width() - gaitBarPositionX_ - gaitBarMarginRight_);
    int gaitBarPositionY = gaitBarMarginTopBottom_;
    for (int i = 0; i < 4; ++i) {
      QRect qRect(gaitBarPositionX_, gaitBarPositionY,
                  gaitBarWidth, gaitBarHeight_);
      painter.fillRect(qRect, gaitBarColor_[i]);
      gaitBarPositionY += gaitBarMarginTopBottom_ + gaitBarHeight_;
    }

    // Draw swing phases.
    boxLength = gaitBarWidth / nrCycles_;
    for (int j = 0; j < 4; j++) {
      int iLeg = j;
      for (int i = 0; i < nrCycles_; i++) {
        const double scale    = gpContainer_[i].getDuration() / totalDuration_;
        double intervalStart  = gpContainer_[i].getStartPhase();
        double intervalEnd    = intervalStart;
        const bool isGrounded = gpContainer_[i].getFootTouchDownPhase(iLeg) >= gpContainer_[i].getFootLiftOffPhase(iLeg);

        if (isGrounded) {
          intervalStart += gpContainer_[i].getFootLiftOffPhase(iLeg) * scale;
          intervalEnd   += gpContainer_[i].getFootTouchDownPhase(iLeg) * scale;
        } else {
          intervalEnd   += gpContainer_[i].getFootTouchDownPhase(iLeg) * scale;
        }

        paintWhiteBar(painter, intervalStart, intervalEnd, gaitBarPositionY, i, j, boxLength);

        if (!isGrounded) {
          intervalStart += gpContainer_[i].getFootLiftOffPhase(iLeg) * scale;
          intervalEnd = 1.0;
          paintWhiteBar(painter, intervalStart, intervalEnd, gaitBarPositionY, i, j, boxLength);
        }

      }
    }
  }

  // Draw line at the beginning and end of the gait bar.
  for (int k = 0; k <= nrCycles_; ++k) {
    painter.setPen(QPen(Qt::black, 1, Qt::DotLine));
    painter.drawLine(gaitBarPositionX_ + k * boxLength, 0,
                     gaitBarPositionX_ + k * boxLength, lineLength_);
  }

  {
    std::lock_guard<std::mutex> lock(mutexCursorPosition_);

    // Draw current time step line.
    double cursorPositionX = gaitBarPositionX_ + cursorPosition_ * gaitBarWidth;
    painter.setPen(QPen(Qt::black, 2, Qt::SolidLine));
    painter.drawLine((int)cursorPositionX, 0, (int)cursorPositionX,
                     lineLength_);
  }

  // Take time.
//  ros::Duration duration = ros::Time::now() - start;
//  ROS_INFO_STREAM_NAMED(TAG, TAG << " Time: " << duration.toSec());
}

void GaitPatternWidget::paintWhiteBar(
    QPainter& painter,
    double intervalStart, double intervalEnd,
    int gaitBarPositionY, int i, int j,
    int boxLength) {
  boundToRange(&intervalStart, 0, 1.0);
  boundToRange(&intervalEnd, 0, 1.0);

  if (intervalStart != intervalEnd) {
    gaitBarPositionY =
        (j + 1) * gaitBarMarginTopBottom_ + j * gaitBarHeight_;

    double vertexInvervalStartX =
        gaitBarPositionX_ + intervalStart * boxLength * nrCycles_ +
        i * boxLength;
    double vertexIntervalEndX =
        gaitBarPositionX_ + intervalEnd * boxLength * nrCycles_ +
        i * boxLength;

    QRect qRect((int)vertexInvervalStartX,
                gaitBarPositionY,
                (int)(vertexIntervalEndX - vertexInvervalStartX),
                gaitBarHeight_);
    painter.fillRect(qRect, Qt::white);
  }
}

} // namespace
