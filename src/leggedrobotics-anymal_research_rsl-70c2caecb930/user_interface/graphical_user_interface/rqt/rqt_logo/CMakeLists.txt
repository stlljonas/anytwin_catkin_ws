cmake_minimum_required(VERSION 3.3)
project(rqt_logo)

set(CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  COMPONENTS
    ratio_layouted_frame
    rqt_gui
    rqt_gui_cpp
)

find_package(Qt5Widgets REQUIRED)

set(rqt_logo_SRCS
  src/rqt_logo/LogoPlugin.cpp
)

set(rqt_logo_HDRS
)

set(rqt_logo_QT_HDRS
  include/rqt_logo/LogoPlugin.h
)

set(rqt_logo_UIS
  src/rqt_logo/LogoPlugin.ui
)

set(rqt_logo_QRC
  resource/resources.qrc
)

set(ui_INCLUDE_DIR
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/rqt_logo"
)

set(rqt_logo_INCLUDE_DIRECTORIES
  include
  SYSTEM
    ${ui_INCLUDE_DIR}/..
)

if(NOT EXISTS ${ui_INCLUDE_DIR})
  file(MAKE_DIRECTORY ${ui_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
#    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ratio_layouted_frame
    rqt_gui
    rqt_gui_cpp
  DEPENDS
)

# include directories before wrap cpp
include_directories(${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  SYSTEM
    ${Qt5Widgets_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_definitions(${Qt5Widgets_DEFINITIONS})
qt5_wrap_cpp(rqt_logo_MOCS ${rqt_logo_QT_HDRS})
qt5_add_resources(rqt_logo_RCC ${rqt_logo_QRC})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${ui_INCLUDE_DIR})
qt5_wrap_ui(rqt_logo_UIS_H ${rqt_logo_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

add_library(${PROJECT_NAME}
  ${rqt_logo_SRCS}
  ${rqt_logo_HDRS}
  ${rqt_logo_QT_HDRS}
  ${rqt_logo_MOCS}
  ${rqt_logo_UIS_H}
  ${rqt_logo_RCC}
)
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  ${catkin_LIBRARIES}
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
install(PROGRAMS scripts/rqt_logo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)
