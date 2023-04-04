cmake_minimum_required(VERSION 3.5.1)
project(anybot_gazebo_plugin)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  any_gazebo_msgs
  any_hector_gazebo_plugins
  any_measurements_ros
  any_msgs
  basic_filters
  cosmo
  cosmo_ros
  gazebo_plugins
  gazebo_ros
  gazebo_worlds
  geometry_msgs
  message_logger
  robot_utils
  romo_std
  roscpp
  signal_logger
  signal_logger_ros
  std_utils
  urdf
  xacro
)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

# Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Catkin package macro
catkin_package(
  INCLUDE_DIRS
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  DEPENDS
    Boost
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

#############
## Install ##
#############
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif()