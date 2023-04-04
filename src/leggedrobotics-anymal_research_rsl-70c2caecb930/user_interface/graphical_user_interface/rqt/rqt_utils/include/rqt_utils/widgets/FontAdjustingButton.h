#pragma once

// Qt
#include <QPushButton>
#include <QResizeEvent>
#include <QStyle>

namespace rqt_utils {

// https://stackoverflow.com/questions/8052201/increase-button-font-size-when-button-size-is-changing
class FontAdjustingButton : public QPushButton {

Q_OBJECT

public:
  explicit FontAdjustingButton(QWidget *parent = NULL) : QPushButton(parent) {
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    resizeFont();
  }

  void resizeFont() {
//    int button_margin = style()->pixelMetric(QStyle::PM_ButtonMargin);
//    double factorHeight = (height() - 2*button_margin) / (double)fontMetrics().height();
//    double factorWidth = (width() - 2*button_margin) / (double)fontMetrics().width("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
//    double factor = qMax(0.01, qMin(factorHeight, factorWidth));
//    QFont f = font();
//    f.setPointSizeF(f.pointSizeF()*factor);
//    setFont(f);
    QFont f = font();
    f.setPointSizeF(30);
    setFont(f);
  }

protected:
  void resizeEvent(QResizeEvent *event) {
    resizeFont();
  }

};

}
