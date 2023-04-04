/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

#include <rqt_anymal_state_visualizer/DiagnosticComponentBase.h>

namespace rqt_anymal_state_visualizer {

class DiagnosticComponentLlc : public DiagnosticComponentBase {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  DiagnosticComponentLlc();

  DiagnosticComponentLlc(const QString &name, const QString &id, QPoint point,
                         QSize size, const QColor &color,
                         const QColor &selectionColor, const QString &message);

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  void initTableModel(int rows, int cols) override;
};

} // namespace
