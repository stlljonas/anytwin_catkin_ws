/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/DiagnosticComponentBattery.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

DiagnosticComponentBattery::DiagnosticComponentBattery() :
    DiagnosticComponentBattery("", "", QPoint(0, 0), QSize(0, 0),
                               QColor(0, 0, 0), QColor(0, 0, 0), "",
                               "", QPoint(0, 0)) {
}

DiagnosticComponentBattery::DiagnosticComponentBattery(
    const QString &name, const QString &id, QPoint point,
    QSize size, const QColor &color, const QColor &selectionColor,
    const QString &message, const QString &fontPath, QPoint statusPoint)
    : DiagnosticComponentBase(name, id, point, size, color, selectionColor,
                              message),
      statusPoint_(statusPoint) {
  abstractTableModel_ = new TableModelBattery(nullptr);
  initTableModel(17, 1);
  setDataTableModel(abstractTableModel_);
  dataTableView_->setMinimumSize(100, 116);
  setTitleLabel(name);

  text_ = new Text(fontPath, 30);
}

DiagnosticComponentBattery::~DiagnosticComponentBattery() {
    delete text_;
    delete batteryRectangle_;
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void DiagnosticComponentBattery::initTableModel(int rows, int cols) {
  abstractTableModel_->init(rows, cols);
}

void DiagnosticComponentBattery::setPercentage(double percentage) {
  if (percentage < 0.0) {
    percentage_ = 0.0;
  } else if (percentage > 1.0) {
    percentage_ = 1.0;
  } else {
    percentage_ = percentage;
  }

  if (percentage_ != previousPercentage_) {
    // Set new color.
    if (percentage_ < 0.20) {
      batteryColor_ =  QColor(255, 0, 0, 255);
    } else if (percentage_ < 0.35) {
      batteryColor_ = QColor(255, 120, 0, 255);
    } else {
      batteryColor_ = QColor(17, 171, 30, 255);
    }
    // Set new rectangle.
    if (batteryRectangle_ == nullptr) {
      batteryRectangle_ = new Rectangle(0, 10, 10, 0);
    }
    // TODO subtract the top of the battery.
    batteryRectangle_->setRectangle(
        point().x(), point().y() + size().height() -
                     (int)std::lround(percentage_ * size().height()),
        point().x() + size().width(), point().y() + size().height());
  }

  previousPercentage_ = percentage_;
}

void DiagnosticComponentBattery::setChargingState(int state) {
  chargingState_ = state;
}

void DiagnosticComponentBattery::drawBatteryLevel(QPainter *painter) {
  if (batteryRectangle_ == nullptr) {
    return;
  }
  batteryRectangle_->draw(painter, batteryColor_);
}

void DiagnosticComponentBattery::drawChargingState(QPainter *painter) {
  if (text_ == nullptr) {
    return;
  }
  QString state;
  switch (chargingState_) {
    case BatteryChargingState::UNKNOWN:
      state = "?";
      break;
    case BatteryChargingState::CHARGING:
      state = "C";
      break;
    case BatteryChargingState::NOT_CHARGING:
      state = "";
      break;
    default:
      state = "";
  }
  text_->renderText(painter, state, statusPoint_.x(), statusPoint_.y(),
                    QColor(0, 0, 0));
}

} // namespace
