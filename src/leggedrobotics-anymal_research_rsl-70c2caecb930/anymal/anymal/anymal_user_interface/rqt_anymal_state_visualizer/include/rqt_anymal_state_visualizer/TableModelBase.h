/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

#include <mutex>
#include <QAbstractTableModel>

namespace rqt_anymal_state_visualizer {

class TableModelBase : public QAbstractTableModel {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit TableModelBase(QObject *parent);

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  int rowCount(const QModelIndex &parent = QModelIndex()) const override;

  int columnCount(const QModelIndex &parent = QModelIndex()) const override;

  QVariant data(const QModelIndex &index,
                int role = Qt::DisplayRole) const override;

  virtual QVariant headerData(int section, Qt::Orientation orientation,
                              int role) const = 0;

  virtual void setGridData(int col, int row, QString data);

  virtual std::string getGridData(int col, int row);

  virtual void init(int rows, int cols);

  virtual void addRow(QString name);

private:

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  QString **gridData_ = nullptr /* [REPORTMODEL_ROWS][REPORTMODEL_COLS] */;

  int cols_ = 0;
  int rows_ = 0;

  mutable std::mutex mutex_;
};

} // namespace
