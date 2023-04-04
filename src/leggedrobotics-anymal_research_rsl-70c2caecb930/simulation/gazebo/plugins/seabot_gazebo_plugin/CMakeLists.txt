cmake_minimum_required(VERSION 3.5.1)
project(seabot_gazebo_plugin)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  any_msgs
  anybot_gazebo_plugin
  cosmo
  cosmo_ros
  message_logger
  robot_utils
  romo_std
  roscpp
  series_elastic_actuator
  series_elastic_actuator_msgs
  series_elastic_actuator_ros
  series_elastic_actuator_sim
  signal_logger
  signal_logger_ros
  std_utils
  urdf
  xacro
)

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
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

#############
## Install ##
#############
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif()