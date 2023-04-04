# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(xsensmt_cosmo)

# Set compiler flags
add_definitions(-std=c++11 -Wall -Werror)

#set(DEBUG_COSMO true)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    any_measurements_ros
    any_node
    cosmo_ros
    diagnostic_updater
    roscpp
    sensor_msgs
    std_msgs
    xsensmt_interface
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
    any_measurements_ros
    any_node
    cosmo_ros
    diagnostic_updater
    roscpp
    sensor_msgs
    std_msgs
    xsensmt_interface
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
add_library(
  ${PROJECT_NAME}
  src/XsensmtCosmoNode.cpp
  src/XsensmtTriggeringCosmoNode.cpp
)


#normal imu mesurement
add_executable(
  ${PROJECT_NAME}_node
  src/xsensmt_cosmo_node.cpp
)
add_dependencies(
  ${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#imu with the trigger measurement
add_executable(
  ${PROJECT_NAME}_triggering_node
  src/xsensmt_triggering_cosmo_node.cpp
)
add_dependencies(
  ${PROJECT_NAME}_triggering_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_triggering_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#cosmo subscriber test
add_executable(
  ${PROJECT_NAME}_test_node
  test/xsensmt_test_cosmo_node.cpp
)
add_dependencies(
  ${PROJECT_NAME}_test_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_test_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_triggering_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



