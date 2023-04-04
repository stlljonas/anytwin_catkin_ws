/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_anymal_state_visualizer/Text.h"

namespace rqt_anymal_state_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

Text::Text(const QString &fontPath, int pixelSize) :
    pixelSize_(pixelSize) {

  if (!fontPath.isEmpty()) {
    int id = QFontDatabase::addApplicationFont(fontPath);
    QStringList fontFamilies = QFontDatabase::applicationFontFamilies(id);
    font_ = QFont(fontFamilies[0]);
  } else {
    font_ = QFont("Arial");
  }
  font_.setPixelSize(pixelSize);
}

Text::~Text() {

}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void Text::renderText(QPainter *painter, const QString &text,
                      int x, int y, const QColor &color) {
  painter->setPen(QPen(color, 1, Qt::SolidLine));
  painter->setFont(font_);
  painter->drawText(x, y, text);
}

/* ========================================================================== */
/* Methods                                                                    */
/* ========================================================================== */

} // namespace
