cmake_minimum_required(VERSION 3.5.1)
project(rqt_gait_switcher)

# Set compiler flags
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    param_io
    roscpp
    rqt_gui
    rqt_gui_cpp
    motion_generation_msgs
    std_srvs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

# Source files
set(SRCS
  src/GaitSwitcherPlugin.cpp
)

# Header files containing Q_OBJECTs and need to be wrapped by qt
set(HDRS_QT
  include/rqt_gait_switcher/GaitSwitcherPlugin.hpp
)

# UI files
set(UIS
  resource/gait_switcher_plugin.ui
)

set(ui_INCLUDE_DIR
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

set(INCLUDE_DIRECTORIES
  include
  ${ui_INCLUDE_DIR}/..
)

if(NOT EXISTS ${ui_INCLUDE_DIR})
  file(MAKE_DIRECTORY ${ui_INCLUDE_DIR})
endif()

catkin_package(
  CATKIN_DEPENDS
    param_io
    roscpp 
    rqt_gui 
    rqt_gui_cpp
    motion_generation_msgs
    std_srvs
)

include_directories(
  ${INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  include_directories(${Qt5Widgets_INCLUDE_DIRS})
  add_definitions(${Qt5Widgets_DEFINITIONS})
  qt5_wrap_cpp(MOCS ${HDRS_QT})
else()
  qt4_wrap_cpp(MOCS ${HDRS_QT})
endif()

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${ui_INCLUDE_DIR})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(UIS_H ${UIS})
else()
  qt4_wrap_ui(UIS_H ${UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

add_library(${PROJECT_NAME}
  ${SRCS}
  ${MOCS}
  ${UIS_H}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

