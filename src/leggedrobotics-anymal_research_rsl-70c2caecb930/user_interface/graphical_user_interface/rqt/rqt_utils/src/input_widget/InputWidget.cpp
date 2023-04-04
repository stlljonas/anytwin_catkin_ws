#include <QLabel>
#include <QLayout>
#include <QWidget>
#include <QComboBox>

#include <rqt_utils/input_widget/InputWidget.h>

#include <iostream>
#include <typeinfo>

namespace rqt_utils {

  //! Return true if input was handled
  bool InputWidget::processInput(int input) {
    // Assert wrong layout
    if(getLayout() == nullptr) { return false; }

    // Get current widget
    auto currentItem = getLayout()->itemAt(currentIndex_);
    auto currentWidget = currentItem != nullptr ? currentItem->widget() : nullptr;
    const auto inputEnum = static_cast<Input>(input);
    auto currentInputWidget = dynamic_cast<InputWidget*>(currentWidget);

    // Check if the current widget will react to the input
    if (currentInputWidget != nullptr && currentInputWidget->processInput(input)){
      return true;
    }

    // If current widget is a combobox -> handle
    if(auto currentCombobox = dynamic_cast<QComboBox*>(currentWidget)) {
      if(rqt_utils::input_widget_helpers::handleCombobox(currentCombobox, inputEnum)) {
        return true;
      }
    }

    // Find next relevant Item
    int nextIdx = nextItem(getLayout(), currentIndex_, inputEnum);

    if (nextIdx != -1) {
      // Next widget (must exist when nextIdx is not -1 )
      currentIndex_ = nextIdx;
      auto newWidget = getLayout()->itemAt(currentIndex_)->widget();

      // Check if new widget is input widget
      if (auto newInputWidget = dynamic_cast<InputWidget*>(newWidget)) {
        // Handle siblings with same dynamic type but only if they are not inputwidget bases
        if (typeid(*newWidget) == typeid(*currentWidget) && typeid(*currentWidget) != typeid(InputWidget)) {
          newInputWidget->setCurrentIndex(currentInputWidget->getCurrentIndex());
        }
        newInputWidget->focusCurrent();
      } else {
        getLayout()->itemAt(currentIndex_)->widget()->setFocus();
      }
      return true;
    }
    return false;
  }

  bool InputWidget::focusCurrent() {
    // Check if current index is valid otherwise focus next item
    auto layoutItem = getLayout()->itemAt(currentIndex_);
    if (layoutItem != nullptr && layoutItem->widget() != nullptr &&
        !rqt_utils::input_widget_helpers::isIgnoredWidget(layoutItem->widget())) {
      if (auto currentInputWidget = dynamic_cast<InputWidget*>(layoutItem->widget())) {
        currentInputWidget->focusCurrent();
      } else {
        getLayout()->itemAt(currentIndex_)->widget()->setFocus();
      }
      return true;
    }
    return false;
  }

}  // namespace rqt_utils
