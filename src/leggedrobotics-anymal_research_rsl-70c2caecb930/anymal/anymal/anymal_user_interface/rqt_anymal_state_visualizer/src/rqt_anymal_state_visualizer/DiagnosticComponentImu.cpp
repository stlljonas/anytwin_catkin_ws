/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/DiagnosticComponentImu.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

DiagnosticComponentImu::DiagnosticComponentImu() :
    DiagnosticComponentImu("", "", QPoint(0, 0), QSize(0, 0), QColor(0, 0, 0),
                           QColor(0, 0, 0), "") {
}

DiagnosticComponentImu::DiagnosticComponentImu(
    const QString &name, const QString &id, QPoint point, QSize size,
    const QColor &color, const QColor &selectionColor, const QString &message)
    : DiagnosticComponentBase(name, id, point, size,
                              color, selectionColor, message) {
  abstractTableModel_ = new TableModelImu(0);
  initTableModel(6, 1);
  setDataTableModel(abstractTableModel_);
  dataTableView_->setMinimumSize(100, 135);
  setTitleLabel(name);
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void DiagnosticComponentImu::initTableModel(int rows, int cols) {
  abstractTableModel_->init(rows, cols);
}

} // namespace
