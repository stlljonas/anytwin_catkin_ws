#pragma once

#include <QLabel>
#include <QLayout>
#include <QComboBox>

namespace rqt_utils {

namespace input_widget_helpers {

//! All actions that shall
enum Input { Navigate_Left = 0, Navigate_Right, Navigate_Up, Navigate_Down, Select, Back };

bool isIgnoredWidget(QWidget* widget);

int nextItem(QLayout* layout, int currentIdx, Input direction);

bool handleCombobox(QComboBox* combobox, Input direction);

}  // namespace input_widget_helpers

}  // namespace rqt_utils
