/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/DiagnosticComponentFoot.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

DiagnosticComponentFoot::DiagnosticComponentFoot() :
    DiagnosticComponentFoot("", "", QPoint(0, 0), QSize(0, 0), QColor(0, 0, 0),
                            QColor(0, 0, 0), "") {
}

DiagnosticComponentFoot::DiagnosticComponentFoot(
    const QString &name, const QString &id, QPoint point, QSize size,
    const QColor &color, const QColor &selectionColor, const QString &message)
    : DiagnosticComponentBase(name, id, point, size,
                              color, selectionColor, message) {
  abstractTableModel_ = new TableModelFoot(0);
  initTableModel(4, 1);
  setDataTableModel(abstractTableModel_);
  dataTableView_->setMinimumSize(75, 97);
  setTitleLabel(name);
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

const bool &DiagnosticComponentFoot::isOk() const {
  return isOk_;
}

bool &DiagnosticComponentFoot::isOk() {
  return isOk_;
}

void DiagnosticComponentFoot::initTableModel(int rows, int cols) {
  abstractTableModel_->init(rows, cols);
}

} // namespace
