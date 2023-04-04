# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(xsensmt_ros)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(PACKAGE_DEPENDENCIES
    any_node
    diagnostic_updater
    rosbash
    roscpp
    rospy
    sensor_msgs
    xsensmt_interface
    any_msgs
    nodelet
)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

find_package(PkgConfig)
pkg_check_modules(LIBXSENSMT libxsensmt)

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
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    LIBXSENSMT
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBXSENSMT_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(${PROJECT_NAME}
  src/XsensMTROSInterface.cpp
  src/XsensMTROSTriggeringInterface.cpp
  src/XsensMTROSNodelet.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

############
## EXPORT ##
############
# Install nodelet library
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


