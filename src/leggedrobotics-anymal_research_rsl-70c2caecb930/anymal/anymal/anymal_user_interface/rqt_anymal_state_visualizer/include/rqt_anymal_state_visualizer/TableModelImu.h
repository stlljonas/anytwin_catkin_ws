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

enum imuCols {
  IMU_VALUE = 0,
};

enum imuRows {
  IMU_LX = 0,
  IMU_LY = 1,
  IMU_LZ = 2,
  IMU_AX = 3,
  IMU_AY = 4,
  IMU_AZ = 5,
};

class TableModelImu : public TableModelBase {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit TableModelImu(QObject *parent);

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  QVariant headerData(int section, Qt::Orientation orientation,
                      int role) const override;
};

} // namespace
