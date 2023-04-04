cmake_minimum_required(VERSION 3.5.1)
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 NEW)
endif()

project(center_frame_tool)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
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
   LIBRARIES center_frame_tool
   CATKIN_DEPENDS rviz
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ library
add_library(center_frame_tool
  src/center_frame_tool.cpp
)

add_dependencies(center_frame_tool
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(center_frame_tool
  ${catkin_LIBRARIES}
)

if(rviz_QT_VERSION VERSION_GREATER "5.0.0")
  qt5_use_modules(center_frame_tool Widgets)
  target_link_libraries(center_frame_tool Qt5::Widgets)
else()
  target_link_libraries(center_frame_tool ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

## Install rules

install(TARGETS center_frame_tool
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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
