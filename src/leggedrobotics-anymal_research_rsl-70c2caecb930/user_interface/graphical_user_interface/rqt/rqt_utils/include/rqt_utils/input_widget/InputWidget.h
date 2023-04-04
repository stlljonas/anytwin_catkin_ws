#pragma once

#include <QLabel>
#include <QLayout>
#include <QWidget>

#include <rqt_utils/input_widget/input_widget_helpers.h>

#include <iostream>
#include <typeinfo>

namespace rqt_utils {

class InputWidget : public QWidget {
  Q_OBJECT;
  Q_ENUMS(Input);

 public:
  explicit InputWidget(QWidget* parent) : QWidget(parent) {}
  using Input = input_widget_helpers::Input;

  //! Return true if input was handled
  virtual bool processInput(int input);
  virtual bool focusCurrent();
  virtual QLayout* getLayout() { return this->layout(); }


  int getCurrentIndex() { return currentIndex_; }
  void setCurrentIndex(int currentIndex) { currentIndex_ = currentIndex; }

 protected:
  int currentIndex_{0};
};

}  // namespace rqt_utils
