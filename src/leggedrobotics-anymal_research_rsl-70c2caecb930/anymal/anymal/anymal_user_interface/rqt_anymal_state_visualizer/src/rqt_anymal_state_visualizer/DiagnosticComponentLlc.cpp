/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/DiagnosticComponentLlc.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

DiagnosticComponentLlc::DiagnosticComponentLlc() {
  setTitleLabel("");
}

DiagnosticComponentLlc::DiagnosticComponentLlc(
    const QString &name, const QString &id, QPoint point, QSize size,
    const QColor &color, const QColor &selectionColor, const QString &message)
    : DiagnosticComponentBase(name, id, point, size, color, selectionColor,
                              message) {
  setTitleLabel(name);
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void DiagnosticComponentLlc::initTableModel(int rows, int cols) {

}

} // namespace
