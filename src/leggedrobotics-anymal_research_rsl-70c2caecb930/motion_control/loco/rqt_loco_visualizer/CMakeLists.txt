cmake_minimum_required(VERSION 3.5.1)
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 NEW)
endif()

project(rqt_loco_visualizer)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  anymal_msgs
  roscpp
  rqt_gui
  rqt_gui_cpp
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

###########
# sources #
###########
set(rqt_loco_visualizer_SRCS
  src/rqt_loco_visualizer/LocoVisualizerPlugin.cpp
  src/rqt_loco_visualizer/GaitPatternWidget.cpp
  src/rqt_loco_visualizer/GaitPatternContainer.cpp
)

###########
# headers #
###########
set(rqt_loco_visualizer_HDRS_QT
  include/rqt_loco_visualizer/LocoVisualizerPlugin.h
  include/rqt_loco_visualizer/GaitPatternWidget.h
)
set(rqt_loco_visualizer_HDRS
  include/rqt_loco_visualizer/GaitPatternContainer.h
)

############
# ui files #
############
set(rqt_loco_visualizer_UIS
  src/rqt_loco_visualizer/LocoVisualizerPlugin.ui
)

set(ui_INCLUDE_DIR
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

set(rqt_loco_visualizer_INCLUDE_DIRECTORIES
  include
  include/rqt_loco_visualizer
  ${ui_INCLUDE_DIR}
)

if(NOT EXISTS ${ui_INCLUDE_DIR})
  file(MAKE_DIRECTORY ${ui_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS
    ${rqt_loco_visualizer_INCLUDE_DIRECTORIES}
  LIBRARIES
    rqt_loco_visualizer
  CATKIN_DEPENDS
    anymal_msgs
    roscpp
    rqt_gui
    rqt_gui_cpp
  DEPENDS
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_loco_visualizer_MOCS ${rqt_loco_visualizer_HDRS_QT})
else()
  qt4_wrap_cpp(rqt_loco_visualizer_MOCS ${rqt_loco_visualizer_HDRS_QT})
endif()

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${ui_INCLUDE_DIR})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(rqt_loco_visualizer_UIS_H ${rqt_loco_visualizer_UIS})
else()
  qt4_wrap_ui(rqt_loco_visualizer_UIS_H ${rqt_loco_visualizer_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(
  ${rqt_loco_visualizer_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

add_library(rqt_loco_visualizer
  ${rqt_loco_visualizer_SRCS}
  ${rqt_loco_visualizer_HDRS_QT}
  ${rqt_loco_visualizer_HDRS}
  ${rqt_loco_visualizer_MOCS}
  ${rqt_loco_visualizer_UIS_H}
)

#add_dependencies(rqt_loco_visualizer starleth_msgs_gencpp)
add_dependencies(rqt_loco_visualizer ${catkin_EXPORTED_TARGETS})

target_link_libraries(rqt_loco_visualizer
  ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_use_modules(${PROJECT_NAME} Widgets)
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()


find_package(class_loader)
class_loader_hide_library_symbols(rqt_loco_visualizer)


install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS rqt_loco_visualizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_loco_visualizer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ${ui_INCLUDE_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
