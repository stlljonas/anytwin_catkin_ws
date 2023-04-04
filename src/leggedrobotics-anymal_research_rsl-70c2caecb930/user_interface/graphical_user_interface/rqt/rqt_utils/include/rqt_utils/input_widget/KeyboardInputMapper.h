#pragma once

#include <QSignalMapper>
#include <QWidget>

#include <rqt_utils/input_widget/InputMapper.h>

namespace rqt_utils {

class KeyboardInputMapper : public InputMapper {
  Q_OBJECT;

 public:
  //! Constructor
  explicit KeyboardInputMapper(InputWidget *parent);
  ~KeyboardInputMapper() override = default;

  void addKeyboardShortcut(QSignalMapper *signalMapper, QKeySequence shortcut, Input mappedKey);
  void connectInputMappings() override;
};

}  // namespace
