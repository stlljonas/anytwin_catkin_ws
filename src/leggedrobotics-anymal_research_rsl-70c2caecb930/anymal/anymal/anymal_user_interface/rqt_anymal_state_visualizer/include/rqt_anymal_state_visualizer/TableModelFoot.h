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

enum footCols {
  FOOT_FORCE = 0,
};

enum footRows {
  FOOT_FX = 0,
  FOOT_FY = 1,
  FOOT_FZ = 2,
  FOOT_NORM = 3,
};

class TableModelFoot : public TableModelBase {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit TableModelFoot(QObject *parent);

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  QVariant headerData(int section, Qt::Orientation orientation,
                      int role) const override;
};

} // namespace
