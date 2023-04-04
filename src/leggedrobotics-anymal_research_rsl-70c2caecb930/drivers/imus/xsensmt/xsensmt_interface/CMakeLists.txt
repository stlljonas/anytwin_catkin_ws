# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(xsensmt_interface)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    message_logger
)

find_package(PkgConfig)
pkg_check_modules(LIBXSENSMT libxsensmt REQUIRED)

# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    ${LIBXSENSMT_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    ${LIBXSENSMT_LIBRARIES}
  CATKIN_DEPENDS
    message_logger
  DEPENDS
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBXSENSMT_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(${PROJECT_NAME}
  src/XsensMTInterface.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LIBXSENSMT_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)