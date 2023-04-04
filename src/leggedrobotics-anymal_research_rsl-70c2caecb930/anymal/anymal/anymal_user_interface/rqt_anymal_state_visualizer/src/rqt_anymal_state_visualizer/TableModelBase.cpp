/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/TableModelBase.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

TableModelBase::TableModelBase(QObject *parent)
    : QAbstractTableModel(parent) {

}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void TableModelBase::init(int rows, int cols) {

  rows_ = rows;
  cols_ = cols;

  gridData_ = new QString *[rows];

  for (int i = 0; i < rows; i++) {
    gridData_[i] = new QString[cols];
  }
}

void TableModelBase::addRow(QString name) {

  QString **gridData__old = NULL;
  gridData__old = new QString *[rows_];
  for (int i = 0; i < rows_; i++) {
    gridData__old[i] = new QString[cols_];
  }
  // copy data
  for (int i = 0; i < rows_; i++) {
    for (int j = 0; j < cols_; j++) {
      gridData__old[i][j] = gridData_[i][j];
    }
  }
  // delete
  for (int i = 0; i < rows_; i++) {
    delete[] gridData_[i];
  }
  delete[] gridData_;

  // add row
  insertRow(rowCount());
  rows_++;
  gridData_ = new QString *[rows_];
  for (int i = 0; i < rows_; i++) {
    gridData_[i] = new QString[cols_];
  }

  // copy old data to new
  for (int i = 0; i < rows_ - 1; i++) {
    for (int j = 0; j < cols_; j++) {
      gridData_[i][j] = gridData__old[i][j];
    }
  }
  // name new row
  gridData_[rows_ - 1][0] = name;
  QModelIndex idx = this->index(rows_ - 1, 0);
  emit dataChanged(idx, idx);
  emit layoutChanged();

}

int TableModelBase::rowCount(const QModelIndex & /*parent*/) const {
  return rows_;
}

int TableModelBase::columnCount(const QModelIndex & /*parent*/) const {
  return cols_;
}

QVariant TableModelBase::data(const QModelIndex &index, int role) const {
  std::lock_guard<std::mutex> lock_guard(mutex_);

  if (role == Qt::DisplayRole) {
    return gridData_[index.row()][index.column()];
  }
  return QVariant();
}

void TableModelBase::setGridData(int col, int row, QString data) {
  {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    gridData_[row][col] = data;
  }

  QModelIndex idx = this->index(row, col);

  emit dataChanged(idx, idx);
}

std::string TableModelBase::getGridData(int col, int row) {

  return gridData_[row][col].toStdString();
}

} // namespace
