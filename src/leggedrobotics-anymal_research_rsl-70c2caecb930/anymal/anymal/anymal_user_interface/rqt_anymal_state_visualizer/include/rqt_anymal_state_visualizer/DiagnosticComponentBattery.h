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
#include <rqt_anymal_state_visualizer/TableModelBattery.h>
#include <rqt_anymal_state_visualizer/Text.h>

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Enums                                                                      */
/* ========================================================================== */

enum BatteryChargingState {
  UNKNOWN = 0,
  CHARGING = 1,
  NOT_CHARGING = 2,
};

class DiagnosticComponentBattery : public DiagnosticComponentBase {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  DiagnosticComponentBattery();

  DiagnosticComponentBattery(const QString &name, const QString &id,
                             QPoint point, QSize size, const QColor &color,
                             const QColor &selectionColor,
                             const QString &message, const QString &fontPath,
                             QPoint statusPoint);

  ~DiagnosticComponentBattery();

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  void initTableModel(int rows, int cols) override;

  void setPercentage(double percentage);

  void setChargingState(int state);

  void drawBatteryLevel(QPainter *painter);

  void drawChargingState(QPainter *painter);

private:

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  Text* text_ = nullptr;
  Rectangle *batteryRectangle_ = nullptr;

  QPoint statusPoint_;

  QColor batteryColor_;

  double percentage_ = 0.0;
  double previousPercentage_ = -1.0;

  int chargingState_ = BatteryChargingState::UNKNOWN;
};

} // namespace
