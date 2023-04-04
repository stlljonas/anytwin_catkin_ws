/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/TableModelEstimator.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

TableModelEstimator::TableModelEstimator(QObject *parent)
    : TableModelBase(parent) {

}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

QVariant TableModelEstimator::headerData(int section,
                                         Qt::Orientation orientation,
                                         int role) const {

  if (role == Qt::DisplayRole) {
    if (orientation == Qt::Horizontal) {
      switch (section) {
        case EST_ODOM:
          return QString("Odom");
        case EST_MAP:
          return QString("Map");
      }
    }
    if (orientation == Qt::Vertical) {
      switch (section) {
        case EST_PX:
          return QString("Pos x");
        case EST_PY:
          return QString("Pos y");
        case EST_PZ:
          return QString("Pos z");
        case EST_YAW:
          return QString("Yaw");
        case EST_PITCH:
          return QString("Pitch");
        case EST_ROLL:
          return QString("Roll");
      }
    }
  }
  return QVariant();
}

} // namespace
