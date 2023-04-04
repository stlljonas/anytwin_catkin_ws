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
#include <rqt_anymal_state_visualizer/TableModelEstimator.h>

namespace rqt_anymal_state_visualizer {

class DiagnosticComponentEstimator : public DiagnosticComponentBase {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  DiagnosticComponentEstimator();

  DiagnosticComponentEstimator(const QString &name, const QString &id,
                               QPoint point, QSize size, const QColor &color,
                               const QColor &selectionColor,
                               const QString &message);

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  virtual const bool &isOk() const;

  virtual bool &isOk();

  void initTableModel(int rows, int cols) override;

protected:

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  bool isOk_ = true;
};

} // namespace
