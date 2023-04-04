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
#include <rqt_anymal_state_visualizer/Rectangle.h>
#include <rqt_anymal_state_visualizer/Circle.h>
// std
#include <deque>
// QT
#include <QColor>
#include <QSize>
#include <QPoint>
#include <QFont>
#include <QLabel>
#include <QTableView>
#include <QHeaderView>
#include <QAbstractTableModel>
// GL
#include <GL/glut.h>
// ros
#include <ros/time.h>

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Enums                                                                      */
/* ========================================================================== */

enum levels {
  OK = 0,
  WARN = 1,
  ERROR = 2
};

/* ========================================================================== */
/* Structs                                                                    */
/* ========================================================================== */

struct message_t {
  ros::Time time;
  QString message;
  int level;
  QString id;
  int statusword;

  bool compare(uint16_t statusword) {
    return this->statusword == (int)statusword;
  }
};

class DiagnosticComponentBase {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  DiagnosticComponentBase();

  DiagnosticComponentBase(const QString &name, const QString &id, QPoint point,
                          QSize size, QColor color, QColor selectionColor,
                          const QString &message);

  virtual ~DiagnosticComponentBase();

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  virtual const QString &name() const;

  virtual QString &name();

  virtual const QString &id() const;

  virtual QString &id();

  virtual const QString &message() const;

  virtual QString &message();

  virtual const QColor &color() const;

  virtual QColor &color();

  virtual const QSize &size() const;

  virtual QSize &size();

  virtual const QPoint &point() const;

  virtual QPoint &point();

  virtual void pushMessage(const QString &message, int level,
                           const ros::Time &time, int statusword = 0);

  virtual std::deque<message_t> getMessages();

  virtual message_t getLatestMessage();

  virtual void updateLatestMessageTime(ros::Time time);

  virtual unsigned int getNumberOfMessages();

  virtual void drawCircle(QPainter *painter, const QColor &color);

  virtual void drawCircleBorder(QPainter *painter, const QColor &color);

  virtual void drawSelectionCircle(QPainter *painter);

  virtual void drawRectangle(QPainter *painter, const QColor &color);

  virtual void drawDiagnosticRectangle(QPainter *painter);

  virtual void drawRectangleBorder(QPainter *painter, const QColor &color);

  virtual void drawSelectionRectangleBorder(QPainter *painter);

  virtual void setTitleLabel(const QString &str);

  virtual QLabel *getTitleLabel();

  virtual void initTableModel(int rows, int cols) = 0;

  virtual TableModelBase *getTableModel();

  virtual void setDataTableModel(QAbstractTableModel *tableModel);

  virtual QTableView *getDataTable();

protected:

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  QString name_;
  QString id_;

  QString message_;
  QColor color_;
  QColor selectionColor_;
  QSize size_;
  QPoint point_;

  Rectangle *rectangle_;
  Circle *circle_;

  const int messagesDequeSize_ = 20;
  std::deque<message_t> messages_;

  QLabel *titleLabel_ = nullptr;
  QTableView *dataTableView_ = nullptr;
  TableModelBase *abstractTableModel_ = nullptr;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

  virtual void init();
};

} // namespace
