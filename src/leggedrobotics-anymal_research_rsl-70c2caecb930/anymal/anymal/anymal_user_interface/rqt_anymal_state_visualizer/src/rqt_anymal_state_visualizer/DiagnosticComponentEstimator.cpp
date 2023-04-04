/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/DiagnosticComponentEstimator.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

DiagnosticComponentEstimator::DiagnosticComponentEstimator() :
    DiagnosticComponentEstimator("", "", QPoint(0, 0), QSize(0, 0),
                                 QColor(0, 0, 0), QColor(0, 0, 0), "") {
}

DiagnosticComponentEstimator::DiagnosticComponentEstimator(
    const QString &name, const QString &id, QPoint point, QSize size,
    const QColor &color, const QColor &selectionColor, const QString &message)
    : DiagnosticComponentBase(name,
                              id, point, size,
                              color, selectionColor, message) {
  abstractTableModel_ = new TableModelEstimator(nullptr);
  initTableModel(6, 2);
  setDataTableModel(abstractTableModel_);
  dataTableView_->setMinimumSize(120, 135);
  setTitleLabel(name);
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

const bool &DiagnosticComponentEstimator::isOk() const {
  return isOk_;
}

bool &DiagnosticComponentEstimator::isOk() {
  return isOk_;
}

void DiagnosticComponentEstimator::initTableModel(int rows, int cols) {
  abstractTableModel_->init(rows, cols);
}

} // namespace
