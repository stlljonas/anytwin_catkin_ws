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
#include <rqt_anymal_state_visualizer/TableModelSea.h>

namespace rqt_anymal_state_visualizer {

class DiagnosticComponentSea : public DiagnosticComponentBase {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  DiagnosticComponentSea();

  DiagnosticComponentSea(const QString &name, const QString &id, QPoint point,
                         QSize size, const QColor &color,
                         const QColor &selectionColor, const QString &message);

  DiagnosticComponentSea(const QString &name, const QString &id, QPoint point,
                         QSize size, const QColor &color,
                         const QColor &selectionColor, const QString &message,
                         QPoint capPoint, QSize capSize,
                         const QColor &capColor, QPoint controlModePoint,
                         const QColor &controlModeColor);

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  virtual void drawCapRectangle();

  virtual const QColor &controlModeColor() const;

  virtual QColor &controlModeColor();

  virtual const QPoint &controlModePoint() const;

  virtual QPoint &controlModePoint();

  virtual void initTableModel(int rows, int cols);

  virtual QString &mode();

  virtual const QString &mode() const;

protected:

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  // blue actuator cap
  QColor capColor_;
  QSize capSize_;
  QPoint capPoint_;

  // control mode string
  QColor controlModeColor_;
  QPoint controlModePoint_;

  QString mode_ = "";
};

} // namespace
