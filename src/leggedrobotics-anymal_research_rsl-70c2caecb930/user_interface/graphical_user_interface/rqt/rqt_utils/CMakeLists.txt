cmake_minimum_required(VERSION 3.5.1)
project(rqt_utils)

find_package(catkin REQUIRED
  rqt_gui_cpp
)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5 COMPONENTS Core Widgets Designer REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

# Flags
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(${PROJECT_NAME}_SRCS
  src/common/QSignalHandler.cpp
  src/input_widget/input_widget_helpers.cpp
  src/input_widget/InputWidget.cpp
  src/input_widget/KeyboardInputMapper.cpp
  src/widgets/LED.cpp

)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/common/QSignalHandler.h
  include/${PROJECT_NAME}/input_widget/InputWidget.h
  include/${PROJECT_NAME}/input_widget/InputMapper.h
  include/${PROJECT_NAME}/input_widget/KeyboardInputMapper.h
  include/${PROJECT_NAME}/widgets/LED.h
  include/${PROJECT_NAME}/widgets/FontAdjustingButton.h
  include/${PROJECT_NAME}/widgets/ResizableWidget.h
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
    ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Designer_INCLUDE_DIRS}
  )
endif()

catkin_package(
  INCLUDE_DIRS
    ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui_cpp
)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
else()
  qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
endif()

include_directories(${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_HDRS}
  ${${PROJECT_NAME}_MOCS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
