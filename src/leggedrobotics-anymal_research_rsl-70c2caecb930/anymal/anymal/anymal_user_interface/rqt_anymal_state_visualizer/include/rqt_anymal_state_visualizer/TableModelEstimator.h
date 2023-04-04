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

enum estimatorCols {
  EST_ODOM = 0,
  EST_MAP = 1,
};

enum estimatorRows {
  EST_PX = 0,
  EST_PY = 1,
  EST_PZ = 2,
  EST_YAW = 3,
  EST_PITCH = 4,
  EST_ROLL = 5,
};

class TableModelEstimator : public TableModelBase {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit TableModelEstimator(QObject *parent);

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  QVariant headerData(int section, Qt::Orientation orientation,
                      int role) const override;
};

} // namespace
