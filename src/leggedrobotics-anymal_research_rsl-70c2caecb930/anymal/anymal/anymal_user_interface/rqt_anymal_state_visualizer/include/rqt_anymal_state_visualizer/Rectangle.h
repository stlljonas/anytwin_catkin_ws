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

namespace rqt_anymal_state_visualizer {

class Rectangle {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  Rectangle(int left, int top, int right, int bottom);

  ~Rectangle();

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  void draw(QPainter *painter, const QColor &color);

  void drawBorder(QPainter *painter, const QColor &color);

  void drawBorder(QPainter *painter, const QColor &color,
                  int left, int top, int right, int bottom);

  void setRectangle(int left, int top, int right, int bottom);

private:

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  bool isInitialized_ = false;

  int left_;
  int top_;
  int right_;
  int bottom_;

  QRect rect_;
};

} // namespace
