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

enum seaCols {
  SEA_STATE = 0,
  SEA_COMMANDED = 1
};

enum seaRows {
  SEA_POSITION = 0,
  SEA_VELOCITY = 1,
  SEA_TORQUE = 2,
  SEA_CURRENT = 3,
};

class TableModelSea : public TableModelBase {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit TableModelSea(QObject *parent);

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  QVariant headerData(int section, Qt::Orientation orientation,
                      int role) const override;
};

} // namespace
