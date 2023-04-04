#pragma once

#include <rqt_utils/input_widget/InputWidget.h>

#include <QWidget>
#include <iostream>

namespace rqt_utils {

class InputMapper : public QObject {
  Q_OBJECT;

 public:
  using Input = InputWidget::Input;

  //! Constructor
  explicit InputMapper(InputWidget *parent) : parent_(parent){};
  ~InputMapper() override = default;

  virtual void connectInputMappings() = 0;

 public slots:
  void processInput(int input) { parent_->processInput(input); }

 protected:
  InputWidget *parent_;
};

}  // namespace
