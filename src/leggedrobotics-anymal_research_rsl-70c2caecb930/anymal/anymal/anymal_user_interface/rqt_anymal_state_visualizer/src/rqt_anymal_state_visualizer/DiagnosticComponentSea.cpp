/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/DiagnosticComponentSea.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

DiagnosticComponentSea::DiagnosticComponentSea() :
    DiagnosticComponentSea("", "", QPoint(0, 0), QSize(0, 0), QColor(0, 0, 0),
                           QColor(0, 0, 0), "") {
}

DiagnosticComponentSea::DiagnosticComponentSea(
    const QString &name, const QString &id, QPoint point, QSize size,
    const QColor &color, const QColor &selectionColor, const QString &message)
    : DiagnosticComponentBase(name, id, point, size, color, selectionColor,
                              message) {
  DiagnosticComponentSea(name, id, point, size, color, selectionColor, message,
                         QPoint(0, 0), QSize(0, 0), QColor(0, 0, 0),
                         QPoint(0, 0), QColor(0, 0, 0));
}

DiagnosticComponentSea::DiagnosticComponentSea(
    const QString &name, const QString &id, QPoint point, QSize size,
    const QColor &color, const QColor &selectionColor, const QString &message,
    QPoint capPoint, QSize capSize, const QColor &capColor,
    QPoint controlModePoint, const QColor &controlModeColor)
    : DiagnosticComponentBase(name, id, point, size, color, selectionColor,
                              message),
      capPoint_(capPoint),
      capSize_(capSize),
      capColor_(capColor),
      controlModePoint_(controlModePoint),
      controlModeColor_(controlModeColor) {
  abstractTableModel_ = new TableModelSea(0);
  initTableModel(4, 2);
  setDataTableModel(abstractTableModel_);
  dataTableView_->setMinimumSize(170, 97);
  setTitleLabel(name);
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void DiagnosticComponentSea::drawCapRectangle() {
//  drawRectangle(capPoint_, capSize_, capColor_);
}

const QColor &DiagnosticComponentSea::controlModeColor() const {
  return controlModeColor_;
}

QColor &DiagnosticComponentSea::controlModeColor() {
  return controlModeColor_;
}

const QPoint &DiagnosticComponentSea::controlModePoint() const {
  return controlModePoint_;
}

QPoint &DiagnosticComponentSea::controlModePoint() {
  return controlModePoint_;
}

void DiagnosticComponentSea::initTableModel(int rows, int cols) {
  abstractTableModel_->init(rows, cols);
}

QString &DiagnosticComponentSea::mode() {
  return mode_;
}

const QString &DiagnosticComponentSea::mode() const {
  return mode_;
}

} // namespace
