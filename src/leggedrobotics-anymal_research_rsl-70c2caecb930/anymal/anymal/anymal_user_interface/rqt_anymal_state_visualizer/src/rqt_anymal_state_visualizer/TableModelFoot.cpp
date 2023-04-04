/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/TableModelFoot.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

TableModelFoot::TableModelFoot(QObject *parent)
    : TableModelBase(parent) {

}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

QVariant TableModelFoot::headerData(int section, Qt::Orientation orientation,
                                    int role) const {

  if (role == Qt::DisplayRole) {
    if (orientation == Qt::Horizontal) {
      switch (section) {
        case FOOT_FORCE:
          return QString("Force");
      }
    }
    if (orientation == Qt::Vertical) {
      switch (section) {
        case FOOT_FX:
          return QString("fx");
        case FOOT_FY:
          return QString("fy");
        case FOOT_FZ:
          return QString("fz");
        case FOOT_NORM:
          return QString("||f||");
      }
    }
  }
  return QVariant();
}

} // namespace
