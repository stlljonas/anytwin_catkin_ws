#include <QAbstractItemView>

#include <rqt_utils/input_widget/input_widget_helpers.h>

namespace rqt_utils {

namespace input_widget_helpers {

bool isIgnoredWidget(QWidget* widget) {
  // Labels are ignored in the navigation
  return (dynamic_cast<QLabel*>(widget) != nullptr);
}

int nextItem(QLayout* layout, int currentIdx, const Input direction) {
  if (layout->count() <= 0) {
    return -1;
  }

  if (auto gridLayout = dynamic_cast<QGridLayout*>(layout)) {
    int r, c, rowSpan, colSpan;
    gridLayout->getItemPosition(currentIdx, &r, &c, &rowSpan, &colSpan);
    QLayoutItem* layoutItem = nullptr;
    unsigned int stepSize = 1;
    while (layoutItem == nullptr || layoutItem->widget() == nullptr || isIgnoredWidget(layoutItem->widget())) {
      switch (direction) {
        case Input::Navigate_Left:
          if (c == (stepSize - 1)) {
            return -1;
          }  // Reached left border
          layoutItem = gridLayout->itemAtPosition(r, c - stepSize);
          break;
        case Input::Navigate_Right:
          if (c == (gridLayout->columnCount() - stepSize)) {
            return -1;
          }  // Reached right border
          layoutItem = gridLayout->itemAtPosition(r, c + stepSize);
          break;
        case Input::Navigate_Up:
          if (r == (stepSize - 1)) {
            return -1;
          }  // Reached upper border
          layoutItem = gridLayout->itemAtPosition(r - stepSize, c);
          break;
        case Input::Navigate_Down:
          if (r == (gridLayout->rowCount() - stepSize)) {
            return -1;
          }  // Reached lower border
          layoutItem = gridLayout->itemAtPosition(r + stepSize, c);
          break;
        default:
          return -1;
      }
      ++stepSize;
    }
    return layout->indexOf(layoutItem->widget());
  } else if (auto vBoxLayout = dynamic_cast<QVBoxLayout*>(layout)) {
    QLayoutItem* layoutItem = nullptr;
    unsigned int stepSize = 1;
    while (layoutItem == nullptr || layoutItem->widget() == nullptr || isIgnoredWidget(layoutItem->widget())) {
      switch (direction) {
        case Input::Navigate_Up:
          if (currentIdx == (stepSize - 1)) {
            return -1;
          }  // Reached upper border
          layoutItem = layout->itemAt(currentIdx - stepSize);
          break;
        case Input::Navigate_Down:
          if (currentIdx == (layout->count() - stepSize)) {
            return -1;
          }  // Reached lower border
          layoutItem = layout->itemAt(currentIdx + stepSize);
          break;
        default:
          return -1;
      }
      ++stepSize;
    }
    return layout->indexOf(layoutItem->widget());
  } else if (auto hBoxLayout = dynamic_cast<QHBoxLayout*>(layout)) {
    QLayoutItem* layoutItem = nullptr;
    unsigned int stepSize = 1;
    while (layoutItem == nullptr || layoutItem->widget() == nullptr || isIgnoredWidget(layoutItem->widget())) {
      switch (direction) {
        case Input::Navigate_Left:
          if (currentIdx == (stepSize - 1)) {
            return -1;
          }  // Reached upper border
          layoutItem = layout->itemAt(currentIdx - stepSize);
          break;
        case Input::Navigate_Right:
          if (currentIdx == (layout->count() - stepSize)) {
            return -1;
          }  // Reached lower border
          layoutItem = layout->itemAt(currentIdx + stepSize);
          break;
        default:
          return -1;
      }
      ++stepSize;
    }
    return layout->indexOf(layoutItem->widget());
  }

  return -1;
}

bool handleCombobox(QComboBox* combobox, const Input direction) {

  if (combobox->view()->isVisible()) {
    if (direction == Input::Navigate_Down) {
      // Select next input in popup
      const auto currentIndex = combobox->view()->currentIndex();
      const auto newIndexRow = (currentIndex.row() + 1) % currentIndex.model()->rowCount();
      const auto newIndex = combobox->view()->model()->index(newIndexRow, currentIndex.column());
      combobox->view()->setCurrentIndex(newIndex);
      return true;
    } else if (direction == Input::Navigate_Up) {
      // Select previous input in popup
      const auto currentIndex = combobox->view()->currentIndex();
      const auto newIndexRow =
          ((currentIndex.row() - 1) + currentIndex.model()->rowCount()) % currentIndex.model()->rowCount();
      const auto newIndex = combobox->view()->model()->index(newIndexRow, currentIndex.column());
      combobox->view()->setCurrentIndex(newIndex);
      return true;
    } else if (direction == Input::Select || direction == Input::Navigate_Right || direction == Input::Navigate_Left) {
      // Close popup and set current index
      combobox->setCurrentIndex(combobox->view()->currentIndex().row());
      combobox->hidePopup();
      // Return only as handled if select button was pressed
      return (direction == Input::Select);
    } else {
      // Back don't applies changes
      combobox->hidePopup();
      return true;
    }
  } else {
    if (direction == Input::Select) {
      combobox->showPopup();
      return true;
    }
  }

  return false;
}

}  // namespace input_widget_helpers

}  // namespace rqt_utils
