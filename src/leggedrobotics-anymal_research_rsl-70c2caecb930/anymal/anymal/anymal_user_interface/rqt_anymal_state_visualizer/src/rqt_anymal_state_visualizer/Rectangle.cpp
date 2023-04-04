/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/Rectangle.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

Rectangle::Rectangle(int left, int top, int right, int bottom) {
  setRectangle(left, top, right, bottom);
}

Rectangle::~Rectangle() {

}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void Rectangle::draw(QPainter *painter, const QColor &color) {
  painter->fillRect(rect_, color);
}

void Rectangle::drawBorder(QPainter *painter, const QColor &color) {
  painter->setPen(QPen(color, 2, Qt::SolidLine));
  painter->drawRect(rect_);
}

void Rectangle::drawBorder(QPainter *painter, const QColor &color,
                           int left, int top, int right, int bottom) {
  // Correct the positions.
  if (left > right) {
    std::swap(left, right);
  }
  if (bottom < top) {
    std::swap(bottom, top);
  }

  QRect rect(QPoint(left, top), QPoint(right, bottom));

  painter->setPen(QPen(color, 1, Qt::SolidLine));
  painter->drawRect(rect);
}

void Rectangle::setRectangle(int left, int top, int right, int bottom) {
  // Correct the positions.
  if (left > right) {
    std::swap(left, right);
  }
  if (bottom < top) {
    std::swap(bottom, top);
  }

  left_ = left;
  top_ = top;
  right_ = right;
  bottom_ = bottom;

  rect_ = QRect(QPoint(left, top), QPoint(right, bottom));

  if (!isInitialized_) {
    isInitialized_ = true;
  }
}

} // namespace
