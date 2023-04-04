/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/DiagnosticComponentBase.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

DiagnosticComponentBase::DiagnosticComponentBase() {
  init();
}

DiagnosticComponentBase::DiagnosticComponentBase(const QString &name,
                                                 const QString &id, QPoint point,
                                                 QSize size, QColor color,
                                                 QColor selectionColor,
                                                 const QString &message)
    : name_(name),
      id_(id),
      point_(point),
      size_(size),
      color_(color),
      selectionColor_(selectionColor),
      message_(message) {
  init();

  rectangle_ = new Rectangle(point.x(), point.y() + size.height(),
                             point.x() + size.width(), point.y());
  circle_ = new Circle(point.x(), point.y() + size.height(),
                       point.x() + size.width(), point.y());
}

DiagnosticComponentBase::~DiagnosticComponentBase() {

}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

void DiagnosticComponentBase::init() {
  titleLabel_ = new QLabel("");
  QFont fontLabel;
  fontLabel.setPointSize(7);
  fontLabel.setWeight(QFont::Bold);
  titleLabel_->setFont(fontLabel);
  dataTableView_ = new QTableView();
  QFont font;
  font.setPointSize(7);
  dataTableView_->setFont(font);

//    dataTableView_->verticalHeader()->setDefaultSectionSize(18);
//    dataTableView_->horizontalHeader()->setDefaultSectionSize(18);

#if QT_VERSION >= 0x050000
  // Qt5 code
  dataTableView_->verticalHeader()->setSectionResizeMode(
  QHeaderView::ResizeToContents);
  dataTableView_->horizontalHeader()->setSectionResizeMode(
  QHeaderView::ResizeToContents);
#else
  // Qt4 code
  dataTableView_->verticalHeader()->setResizeMode(
      QHeaderView::ResizeToContents);
  dataTableView_->horizontalHeader()->setResizeMode(
      QHeaderView::ResizeToContents);
#endif
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

const QString &DiagnosticComponentBase::name() const {
  return name_;
}

QString &DiagnosticComponentBase::name() {
  return name_;
}

const QString &DiagnosticComponentBase::id() const {
  return id_;
}

QString &DiagnosticComponentBase::id() {
  return id_;
}

const QString &DiagnosticComponentBase::message() const {
  return message_;
}

QString &DiagnosticComponentBase::message() {
  return message_;
}

const QColor &DiagnosticComponentBase::color() const {
  return color_;
}

QColor &DiagnosticComponentBase::color() {
  return color_;
}

const QSize &DiagnosticComponentBase::size() const {
  return size_;
}

QSize &DiagnosticComponentBase::size() {
  return size_;
}

const QPoint &DiagnosticComponentBase::point() const {
  return point_;
}

QPoint &DiagnosticComponentBase::point() {
  return point_;
}

void DiagnosticComponentBase::drawSelectionCircle(QPainter *painter) {
  circle_->drawBorder(painter, selectionColor_);
}

void DiagnosticComponentBase::drawSelectionRectangleBorder(QPainter *painter) {
  rectangle_->drawBorder(painter, selectionColor_);
}

void DiagnosticComponentBase::drawDiagnosticRectangle(QPainter *painter) {
  rectangle_->draw(painter, color_);
}

void DiagnosticComponentBase::pushMessage(const QString &message, int level,
                                          const ros::Time &time,
                                          int statusword) {
  message_t message_t1;
  message_t1.message = message;
  message_t1.level = level;
  message_t1.time = time;
  message_t1.id = id();
  message_t1.statusword = statusword;
  messages_.push_front(message_t1);
  if (messages_.size() > messagesDequeSize_) {
    messages_.pop_back();
  }
}

std::deque<message_t> DiagnosticComponentBase::getMessages() {
  return messages_;
}

message_t DiagnosticComponentBase::getLatestMessage() {
  return messages_.front();
}

void DiagnosticComponentBase::updateLatestMessageTime(ros::Time time) {
  messages_.front().time = time;
}

unsigned int DiagnosticComponentBase::getNumberOfMessages() {
  return messages_.size();
}

void DiagnosticComponentBase::setDataTableModel(
    QAbstractTableModel *tableModel) {
  dataTableView_->setModel(tableModel);
}

QTableView *DiagnosticComponentBase::getDataTable() {
  return dataTableView_;
}

void DiagnosticComponentBase::setTitleLabel(const QString &str) {
  titleLabel_->setText(str);
}

QLabel *DiagnosticComponentBase::getTitleLabel() {
  return titleLabel_;
}

TableModelBase *DiagnosticComponentBase::getTableModel() {
  return abstractTableModel_;
}

void DiagnosticComponentBase::drawCircleBorder(QPainter *painter,
                                               const QColor &color) {
  circle_->drawBorder(painter, color);
}

void DiagnosticComponentBase::drawRectangle(QPainter *painter,
                                            const QColor &color) {
  rectangle_->draw(painter, color);
}

void DiagnosticComponentBase::drawRectangleBorder(QPainter *painter,
                                                  const QColor &color) {
  rectangle_->drawBorder(painter, color);
}

void DiagnosticComponentBase::drawCircle(QPainter *painter,
                                         const QColor &color) {
  circle_->draw(painter, color);
}

} // namespace
