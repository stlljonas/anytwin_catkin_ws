#include "rqt_utils/input_widget/KeyboardInputMapper.h"
#include <QShortcut>
#include <QSignalMapper>

namespace rqt_utils {

KeyboardInputMapper::KeyboardInputMapper(InputWidget *parent) : InputMapper(parent) {}

void KeyboardInputMapper::addKeyboardShortcut(QSignalMapper *signalMapper, QKeySequence keySequence, Input input) {
  auto shortcut = new QShortcut(keySequence, parent_);
  shortcut->setContext(Qt::WidgetWithChildrenShortcut);
  QObject::connect(shortcut, SIGNAL(activated()), signalMapper, SLOT(map()));
  signalMapper->setMapping(shortcut, static_cast<std::underlying_type<Input>::type>(input));
}

void KeyboardInputMapper::connectInputMappings() {
  auto signalMapper = new QSignalMapper();

  addKeyboardShortcut(signalMapper, Qt::Key_Left, Input::Navigate_Left);
  addKeyboardShortcut(signalMapper, Qt::Key_Right, Input::Navigate_Right);
  addKeyboardShortcut(signalMapper, Qt::Key_Up, Input::Navigate_Up);
  addKeyboardShortcut(signalMapper, Qt::Key_Down, Input::Navigate_Down);
  addKeyboardShortcut(signalMapper, Qt::Key_S, Input::Select);
  addKeyboardShortcut(signalMapper, Qt::Key_Backspace, Input::Back);

  connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(processInput(int)));
}

}  // namespace
