/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

// ros
#include <ros/ros.h>

#include <QColor>
#include <QFont>
#include <QFontDatabase>
#include <QFont>
#include <QPainter>

namespace rqt_anymal_state_visualizer {

class Text {
public:

  /* ======================================================================== */
  /* Constructor/Destructor                                                   */
  /* ======================================================================== */

  explicit Text(const QString &fontPath, int pixelSize = 16);

  ~Text();

  /* ======================================================================== */
  /* Accessors                                                                */
  /* ======================================================================== */

  void renderText(QPainter *painter, const QString &text,
                  int x, int y, const QColor &color);

private:

  /* ======================================================================== */
  /* Constants                                                                */
  /* ======================================================================== */

  const std::string TAG = "Text";

  /* ======================================================================== */
  /* Variables                                                                */
  /* ======================================================================== */

  int pixelSize_ = 16;

  QFont font_;

  /* ======================================================================== */
  /* Methods                                                                  */
  /* ======================================================================== */

};

} // namespace
