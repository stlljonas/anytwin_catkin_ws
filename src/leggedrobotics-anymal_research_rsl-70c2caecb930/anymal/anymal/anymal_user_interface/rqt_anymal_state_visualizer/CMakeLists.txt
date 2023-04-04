cmake_minimum_required(VERSION 3.5.1)
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 NEW)
endif()

project(rqt_anymal_state_visualizer)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find packages.
find_package(catkin REQUIRED
  COMPONENTS
    anydrive
    anydrive_msgs
    anymal_msgs
    battery_widget
    diagnostic_msgs
    notification
    roscpp
    rqt_gui
    rqt_gui_cpp
    sensor_msgs
    series_elastic_actuator_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if (NOT kindr_FOUND)
#   Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif ()

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

###########
# sources #
###########
set(rqt_anymal_state_visualizer_SRCS
  src/rqt_anymal_state_visualizer/Circle.cpp
  src/rqt_anymal_state_visualizer/DiagnosticComponentBase.cpp
  src/rqt_anymal_state_visualizer/DiagnosticComponentBattery.cpp
  src/rqt_anymal_state_visualizer/DiagnosticComponentEstimator.cpp
  src/rqt_anymal_state_visualizer/DiagnosticComponentFoot.cpp
  src/rqt_anymal_state_visualizer/DiagnosticComponentImu.cpp
  src/rqt_anymal_state_visualizer/DiagnosticComponentLlc.cpp
  src/rqt_anymal_state_visualizer/DiagnosticComponentLmc.cpp
  src/rqt_anymal_state_visualizer/DiagnosticComponentSea.cpp
  src/rqt_anymal_state_visualizer/Rectangle.cpp
  src/rqt_anymal_state_visualizer/TableModelBase.cpp
  src/rqt_anymal_state_visualizer/TableModelBattery.cpp
  src/rqt_anymal_state_visualizer/TableModelEstimator.cpp
  src/rqt_anymal_state_visualizer/TableModelFoot.cpp
  src/rqt_anymal_state_visualizer/TableModelImu.cpp
  src/rqt_anymal_state_visualizer/TableModelSea.cpp
  src/rqt_anymal_state_visualizer/Text.cpp
)
set(rqt_anymal_state_visualizer_SRCS_QT
  src/rqt_anymal_state_visualizer/DiagnosticAnymalWidget.cpp
  src/rqt_anymal_state_visualizer/AnymalStateVisualizerPlugin.cpp
)

###########
# headers #
###########
set(rqt_anymal_state_visualizer_HDRS_QT
  include/rqt_anymal_state_visualizer/DiagnosticAnymalWidget.h
  include/rqt_anymal_state_visualizer/AnymalStateVisualizerPlugin.hpp
)
set(rqt_anymal_state_visualizer_HDRS
  include/rqt_anymal_state_visualizer/Circle.h
  include/rqt_anymal_state_visualizer/DiagnosticComponentBase.h
  include/rqt_anymal_state_visualizer/DiagnosticComponentBattery.h
  include/rqt_anymal_state_visualizer/DiagnosticComponentEstimator.h
  include/rqt_anymal_state_visualizer/DiagnosticComponentFoot.h
  include/rqt_anymal_state_visualizer/DiagnosticComponentImu.h
  include/rqt_anymal_state_visualizer/DiagnosticComponentLlc.h
  include/rqt_anymal_state_visualizer/DiagnosticComponentLmc.h
  include/rqt_anymal_state_visualizer/DiagnosticComponentSea.h
  include/rqt_anymal_state_visualizer/Rectangle.h
  include/rqt_anymal_state_visualizer/TableModelBase.h
  include/rqt_anymal_state_visualizer/TableModelBattery.h
  include/rqt_anymal_state_visualizer/TableModelEstimator.h
  include/rqt_anymal_state_visualizer/TableModelFoot.h
  include/rqt_anymal_state_visualizer/TableModelImu.h
  include/rqt_anymal_state_visualizer/TableModelSea.h
  include/rqt_anymal_state_visualizer/Text.h
)

############
# ui files #
############
set(rqt_anymal_state_visualizer_UIS
  resource/anymal_state_visualizer_plugin.ui
)

set(${PROJECT_NAME}_QRC
  resource/resources.qrc
)

set(ui_INCLUDE_DIR
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  ${ui_INCLUDE_DIR}/..
)

if(NOT EXISTS ${ui_INCLUDE_DIR})
  file(MAKE_DIRECTORY ${ui_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS
    ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  LIBRARIES
    rqt_anymal_state_visualizer
    rqt_anymal_state_visualizer_core
  CATKIN_DEPENDS
    anydrive
    anydrive_msgs
    anymal_msgs
    battery_widget
    diagnostic_msgs
    notification
    anymal_msgs
    roscpp
    rqt_gui
    rqt_gui_cpp
    sensor_msgs
    series_elastic_actuator_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_anymal_state_visualizer_MOCS ${rqt_anymal_state_visualizer_HDRS_QT})
  qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_QRC})
else()
  qt4_wrap_cpp(rqt_anymal_state_visualizer_MOCS ${rqt_anymal_state_visualizer_HDRS_QT})
  qt4_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_QRC})
endif()

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${ui_INCLUDE_DIR})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(rqt_anymal_state_visualizer_UIS_H ${rqt_anymal_state_visualizer_UIS})
else()
  qt4_wrap_ui(rqt_anymal_state_visualizer_UIS_H ${rqt_anymal_state_visualizer_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(
  ${rqt_anymal_state_visualizer_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${kindr_INCLUDE_DIRS}
)

add_library(rqt_anymal_state_visualizer_core
  ${rqt_anymal_state_visualizer_SRCS}
)

add_dependencies(rqt_anymal_state_visualizer_core ${catkin_EXPORTED_TARGETS})

target_link_libraries(rqt_anymal_state_visualizer_core
  ${catkin_LIBRARIES}
)

if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(rqt_anymal_state_visualizer_core Qt5::Widgets)
else()
  target_link_libraries(rqt_anymal_state_visualizer_core ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()


add_library(rqt_anymal_state_visualizer
  ${rqt_anymal_state_visualizer_SRCS_QT}
  ${rqt_anymal_state_visualizer_HDRS_QT}
  ${rqt_anymal_state_visualizer_HDRS}
  ${rqt_anymal_state_visualizer_MOCS}
  ${rqt_anymal_state_visualizer_UIS_H}
  ${${PROJECT_NAME}_RCC}
)

add_dependencies(rqt_anymal_state_visualizer ${catkin_EXPORTED_TARGETS})

target_link_libraries(rqt_anymal_state_visualizer
  ${catkin_LIBRARIES}
  rqt_anymal_state_visualizer_core
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_use_modules(${PROJECT_NAME} Widgets)
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(rqt_anymal_state_visualizer)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.ttf"
  PATTERN "*.png"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING
  PATTERN "*.launch"
)

install(TARGETS rqt_anymal_state_visualizer rqt_anymal_state_visualizer_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_anymal_state_visualizer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ${ui_INCLUDE_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

########
# test #
########
add_executable(testing_anymal_state_visualizer
  src/rqt_anymal_state_visualizer/test/testing.cpp
)
target_link_libraries(testing_anymal_state_visualizer
  ${catkin_LIBRARIES}
)
add_dependencies(testing_anymal_state_visualizer
  ${catkin_EXPORTED_TARGETS}
)

add_executable(testing_anymal_state_visualizer_msgs
  src/rqt_anymal_state_visualizer/test/testing_msgs.cpp
)

target_link_libraries(testing_anymal_state_visualizer_msgs
  ${catkin_LIBRARIES}
)

add_dependencies(testing_anymal_state_visualizer_msgs
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS testing_anymal_state_visualizer testing_anymal_state_visualizer_msgs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
