cmake_minimum_required(VERSION 3.5.1)
project(rqt_anydrive_monitor)

set(CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PACKAGE_DEPENDENCIES
  anydrive
  anydrive_ros
  anydrive_monitor
  rqt_gui
  rqt_gui_cpp
  std_srvs
  yaml_tools
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

find_package(Qt5Widgets REQUIRED)

set(${PROJECT_NAME}_SRCS
  src/${PROJECT_NAME}/AnydriveMonitorPlugin.cpp
  src/${PROJECT_NAME}/AnydriveInterfaceRos.cpp
)

set(${PROJECT_NAME}_HDRS_QT
  include/${PROJECT_NAME}/AnydriveMonitorPlugin.h
  include/${PROJECT_NAME}/AnydriveInterfaceRos.h
)
set(${PROJECT_NAME}_HDRS
)

set(${PROJECT_NAME}_UIS
  src/${PROJECT_NAME}/AnydriveMonitorPlugin.ui
)

set(${PROJECT_NAME}_QRC
  resource/resources.qrc
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
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

# include directories before wrap cpp
include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

add_definitions(${Qt5Widgets_DEFINITIONS})
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS_QT})
qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_QRC})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${ui_INCLUDE_DIR})
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_HDRS_QT}
  ${${PROJECT_NAME}_HDRS}
  ${${PROJECT_NAME}_MOCS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_RCC}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

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
install(DIRECTORY launch resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
