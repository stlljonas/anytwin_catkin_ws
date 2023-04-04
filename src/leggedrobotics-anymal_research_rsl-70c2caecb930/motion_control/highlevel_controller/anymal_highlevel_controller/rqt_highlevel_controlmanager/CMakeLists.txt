cmake_minimum_required(VERSION 3.5.1)
project(rqt_highlevel_controlmanager)

# Set compiler flags
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  any_msgs
  anymal_msgs
  param_io
  rocoma_msgs
  roscpp
  rqt_gui
  rqt_gui_cpp
  std_msgs
  std_srvs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

set(${PROJECT_NAME}_SRCS
  src/rqt_highlevel_controlmanager/HighLevelControlManagerPlugin.cpp
  src/rqt_highlevel_controlmanager/WorkerThreadCalibrateFeet.cpp
  src/rqt_highlevel_controlmanager/WorkerThreadSetController.cpp
  src/rqt_highlevel_controlmanager/WorkerThreadSetMode.cpp
)

# Qt object independent headers.
set(${PROJECT_NAME}_HDRS
)

# Qt object dependent headers (containing signals and/or slots).
set(${PROJECT_NAME}_QT_HDRS
  include/rqt_highlevel_controlmanager/HighLevelControlManagerPlugin.h
  include/rqt_highlevel_controlmanager/WorkerThreadCalibrateFeet.h
  include/rqt_highlevel_controlmanager/WorkerThreadSetController.h
  include/rqt_highlevel_controlmanager/WorkerThreadSetMode.h
)

set(${PROJECT_NAME}_UIS
  resource/HighLevelControlManagerPlugin.ui
)

set(${PROJECT_NAME}_QRC
)

set(ui_INCLUDE_DIR
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  SYSTEM
    ${ui_INCLUDE_DIR}/..
)

if(NOT EXISTS ${ui_INCLUDE_DIR})
  file(MAKE_DIRECTORY ${ui_INCLUDE_DIR})
endif()

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
    ${ui_INCLUDE_DIR}/..
  CATKIN_DEPENDS
    any_msgs
    anymal_msgs
    param_io
    rocoma_msgs
    roscpp rqt_gui
    rqt_gui_cpp
    std_msgs
    std_srvs
)

# include directories before wrap cpp
include_directories(${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

add_definitions(${Qt5Widgets_DEFINITIONS})
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_QT_HDRS})
qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_QRC})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${ui_INCLUDE_DIR})
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_HDRS}
  ${${PROJECT_NAME}_QT_HDRS}
  ${${PROJECT_NAME}_MOCS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_RCC}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_plugin
  src/rqt_highlevel_controlmanager/export_plugin.cpp
)

add_dependencies(${PROJECT_NAME}_plugin
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_plugin
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME}_plugin)


install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.png"
)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  FILES_MATCHING
  PATTERN "*.yaml"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING
  PATTERN "*.launch"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ ${ui_INCLUDE_DIR}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
