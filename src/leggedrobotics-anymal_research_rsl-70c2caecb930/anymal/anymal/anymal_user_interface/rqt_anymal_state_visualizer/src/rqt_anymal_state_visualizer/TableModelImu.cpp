/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/TableModelImu.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

TableModelImu::TableModelImu(QObject *parent)
    : TableModelBase(parent) {

}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

QVariant TableModelImu::headerData(int section, Qt::Orientation orientation,
                                   int role) const {

  if (role == Qt::DisplayRole) {
    if (orientation == Qt::Horizontal) {
      switch (section) {
        case IMU_VALUE:
          return QString("Acceleration");
      }
    }
    if (orientation == Qt::Vertical) {
      switch (section) {
        case IMU_LX:
          return QString("Lin x");
        case IMU_LY:
          return QString("Lin y");
        case IMU_LZ:
          return QString("Lin z");
        case IMU_AX:
          return QString("Ang x");
        case IMU_AY:
          return QString("Ang y");
        case IMU_AZ:
          return QString("Ang z");
      }
    }
  }
  return QVariant();
}

} // namespace
