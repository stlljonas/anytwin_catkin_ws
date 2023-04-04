/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/TableModelBattery.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

TableModelBattery::TableModelBattery(QObject *parent)
    : TableModelBase(parent) {

}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

QVariant TableModelBattery::headerData(int section, Qt::Orientation orientation,
                                       int role) const {

  if (role == Qt::DisplayRole) {
    if (orientation == Qt::Horizontal) {
      switch (section) {
        case BATTERY_VALUE:
          return QString("Values");
      }
    }
    if (orientation == Qt::Vertical) {
      switch (section) {
        case BATTERY_VOLTAGE:
          return QString("Voltage [V]");
        case BATTERY_CURRENT:
          return QString("Current [A]");
        case BATTERY_PERCENTAGE:
          return QString("Percentage [%]");
        case BATTERY_HEALTH:
          return QString("Health");
        case BATTERY_STATUS:
          return QString("Status");
        case BATTERY_CELL_1:
        case BATTERY_CELL_2:
        case BATTERY_CELL_3:
        case BATTERY_CELL_4:
        case BATTERY_CELL_5:
        case BATTERY_CELL_6:
        case BATTERY_CELL_7:
        case BATTERY_CELL_8:
        case BATTERY_CELL_9:
        case BATTERY_CELL_10:
        case BATTERY_CELL_11:
        case BATTERY_CELL_12: {
          QString str = "Cell ";
          str.append(QString::number(section - (BATTERY_CELL_1 - 1))).
              append(" [V]");
          return str;
        }
      }
    }
  }
  return QVariant();
}

} // namespace
