/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/TableModelSea.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

TableModelSea::TableModelSea(QObject *parent)
    : TableModelBase(parent) {

}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

QVariant TableModelSea::headerData(int section, Qt::Orientation orientation,
                                   int role) const {

  if (role == Qt::DisplayRole) {
    if (orientation == Qt::Horizontal) {
      switch (section) {
        case SEA_STATE:
          return QString("State");
        case SEA_COMMANDED:
          return QString("Commanded");
      }
    }
    if (orientation == Qt::Vertical) {
      switch (section) {
        case SEA_POSITION:
          return QString("Position");
        case SEA_VELOCITY:
          return QString("Velocity");
        case SEA_TORQUE:
          return QString("Torque");
        case SEA_CURRENT:
          return QString("Current");
      }
    }
  }
  return QVariant();
}

} // namespace
