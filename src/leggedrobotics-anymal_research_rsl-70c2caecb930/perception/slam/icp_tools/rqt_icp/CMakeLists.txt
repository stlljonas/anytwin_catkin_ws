cmake_minimum_required(VERSION 3.5.1)
project(rqt_icp)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall )

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  any_msgs
  slam_common_msgs
  message_logger
  roscpp
  rqt_gui
  rqt_gui_cpp
  std_srvs
)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()
set(rqt_icp_SRCS
  src/rqt_icp/icp_plugin.cpp
)
set(rqt_icp_HDRS
  include/rqt_icp/icp_plugin.h
)
set(rqt_icp_UIS
  src/rqt_icp/icp_plugin.ui
)
set(rqt_icp_QRC
  resource/application.qrc
)
set(rqt_icp_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)
find_package(cmake_clang_tools QUIET)

# Catkin package macro
catkin_package(
  LIBRARIES
    rqt_icp
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_icp_MOCS ${rqt_icp_HDRS})
  qt5_wrap_ui(rqt_icp_UIS_H ${rqt_icp_UIS})
  qt5_add_resources(rqt_icp_RCC ${rqt_icp_QRC})
else()
  qt4_wrap_cpp(rqt_icp_MOCS ${rqt_icp_HDRS})
  qt4_wrap_ui(rqt_icp_UIS_H ${rqt_icp_UIS})
  qt4_add_resources(rqt_icp_RCC ${rqt_icp_QRC})
endif()

include_directories(
  ${rqt_icp_INCLUDE_DIRECTORIES}
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

########################
## Library definition ##
########################
add_library(rqt_icp
  ${rqt_icp_SRCS}
  ${rqt_icp_HDRS}
  ${rqt_icp_MOCS}
  ${rqt_icp_UIS_H}
  ${rqt_icp_RCC}
)
add_dependencies(rqt_icp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rqt_icp
  ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
  )
else()
  target_link_libraries(${PROJECT_NAME}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
  )
endif()

find_package(class_loader)
class_loader_hide_library_symbols(rqt_icp)

#############
## Install ##
#############
install(
  FILES
    plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  TARGETS
    rqt_icp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  PROGRAMS
    scripts/rqt_icp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY
    resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.png"
  PATTERN "*.svg"
)

#################
## Clang_tools ##
#################
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGETS 
      rqt_icp
    SOURCE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif()
