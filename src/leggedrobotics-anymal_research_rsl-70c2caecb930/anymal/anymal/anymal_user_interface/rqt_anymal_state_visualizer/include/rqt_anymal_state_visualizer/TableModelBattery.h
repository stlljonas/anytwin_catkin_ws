/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

#include <rqt_anymal_state_visualizer/TableModelBase.h>

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Enums                                                                      */
/* ========================================================================== */

enum batteryCols {
  BATTERY_VALUE = 0,
};

enum batteryRows {
  BATTERY_VOLTAGE = 0,
  BATTERY_CURRENT = 1,
  BATTERY_PERCENTAGE = 2,
  BATTERY_HEALTH = 3,
  BATTERY_STATUS = 4,
  BATTERY_CELL_1 = 5,
  BATTERY_CELL_2 = 6,
  BATTERY_CELL_3 = 7,
  BATTERY_CELL_4 = 8,
  BATTERY_CELL_5 = 9,
  BATTERY_CELL_6 = 10,
  BATTERY_CELL_7 = 11,
  BATTERY_CELL_8 = 12,
  BATTERY_CELL_9 = 13,
  BATTERY_CELL_10 = 14,
  BATTERY_CELL_11 = 15,
  BATTERY_CELL_12 = 16,
};

class TableModelBattery : public TableModelBase {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit TableModelBattery(QObject *parent);

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  QVariant headerData(int section, Qt::Orientation orientation,
                      int role) const override;
};

} // namespace
