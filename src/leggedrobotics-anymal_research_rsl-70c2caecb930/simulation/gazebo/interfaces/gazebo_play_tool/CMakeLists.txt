cmake_minimum_required(VERSION 3.5.1)
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 NEW)
endif()

project(gazebo_play_tool)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  rviz
)

## System dependencies are found with CMake's conventions
if(rviz_QT_VERSION VERSION_GREATER "5.0.0")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
   LIBRARIES gazebo_play_tool
   CATKIN_DEPENDS rviz
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ library
add_library(gazebo_play_tool
  src/gazebo_play_tool.cpp
)

add_dependencies(gazebo_play_tool
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(gazebo_play_tool
  ${catkin_LIBRARIES}
)

if(rviz_QT_VERSION VERSION_GREATER "5.0.0")
  qt5_use_modules(gazebo_play_tool Widgets)
  target_link_libraries(gazebo_play_tool Qt5::Widgets)
else()
  target_link_libraries(gazebo_play_tool ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

## Install rules

install(TARGETS gazebo_play_tool
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)