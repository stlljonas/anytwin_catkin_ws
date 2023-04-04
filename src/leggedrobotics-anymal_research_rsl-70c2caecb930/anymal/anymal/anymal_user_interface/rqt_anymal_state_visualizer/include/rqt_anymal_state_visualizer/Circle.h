/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

#include <QColor>
#include <QPainter>
#include <QRect>

namespace rqt_anymal_state_visualizer {

class Circle {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  Circle(int left, int top, int right, int bottom, float thickness = 4.0);

  ~Circle();

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  void draw(QPainter *painter, const QColor &color);

  void drawBorder(QPainter *painter, const QColor &color);

private:

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  const int numSegments_ = 100;

  int left_;
  int top_;
  int right_;
  int bottom_;

  QRect rect_;
};

} // namespace
