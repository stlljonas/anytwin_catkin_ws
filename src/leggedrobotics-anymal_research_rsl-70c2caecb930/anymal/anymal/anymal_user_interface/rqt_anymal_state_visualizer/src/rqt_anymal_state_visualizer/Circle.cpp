/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/Circle.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

Circle::Circle(int left, int top, int right, int bottom, float thickness)
    : left_(left),
      top_(top),
      right_(right),
      bottom_(bottom) {
  rect_ = QRect(QPoint(left, top), QPoint(right, bottom));
}

Circle::~Circle() {

}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void Circle::draw(QPainter *painter, const QColor &color) {
  painter->save();

  painter->setBrush(color);
  painter->drawEllipse(rect_);

  painter->restore();
}

void Circle::drawBorder(QPainter *painter, const QColor &color) {
  painter->setPen(QPen(color, 2, Qt::SolidLine));
  painter->drawEllipse(rect_);
}

} // namespace
