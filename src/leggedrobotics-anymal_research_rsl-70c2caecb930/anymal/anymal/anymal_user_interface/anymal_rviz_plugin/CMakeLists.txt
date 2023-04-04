cmake_minimum_required(VERSION 3.5.1)
project(anymal_rviz_plugin)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Werror -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  anymal_description
  anymal_model
  anymal_model_ros
  anymal_msgs
  rviz
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

# Attempt to find package-based tinyxml
find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml tinyxml REQUIRED)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    tinyxml
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${tinyxml_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_rviz_additions
  src/rviz/properties/button_property.cpp
  include/rviz/properties/button_property.h
)

target_link_libraries(${PROJECT_NAME}_rviz_additions
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

# Headers files need to be included to let the QT MOC thing to work. Apparently.
add_library(${PROJECT_NAME}
  src/anymal_display.cpp
  include/${PROJECT_NAME}/anymal_display.hpp
  src/joint_torque_tree.cpp
  include/${PROJECT_NAME}/joint_torque_tree.hpp
  src/joint_torque.cpp
  include/${PROJECT_NAME}/joint_torque.hpp
  src/contact.cpp
  include/${PROJECT_NAME}/contact_list.hpp
  src/contact_list.cpp
  include/${PROJECT_NAME}/contact.hpp
  src/velocity.cpp
  include/${PROJECT_NAME}/velocity.hpp
  src/contact_polygon.cpp
  include/${PROJECT_NAME}/contact_polygon.hpp
  src/polygon_visual.cpp
  include/${PROJECT_NAME}/polygon_visual.hpp
  src/color_robot.cpp
  include/${PROJECT_NAME}/color_robot.hpp
  src/trace.cpp
  include/${PROJECT_NAME}/trace.hpp
  src/trace_list.cpp
  include/${PROJECT_NAME}/trace_list.hpp

  src/anymal_link_updater.cpp
  src/joint_torque_visual.cpp
  src/contact_visual.cpp
  src/com_projection_visual.cpp
  src/velocity_visual.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${PROJECT_NAME}_rviz_additions
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_rviz_additions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons
)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
      ${PROJECT_NAME}_rviz_additions
  )
endif(cmake_clang_tools_FOUND)
