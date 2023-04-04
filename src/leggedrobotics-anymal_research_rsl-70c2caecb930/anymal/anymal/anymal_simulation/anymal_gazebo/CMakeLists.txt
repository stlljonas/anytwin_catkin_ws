cmake_minimum_required(VERSION 3.5.1)
project(anymal_gazebo)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  anydrive
  anydrive_msgs
  anydrive_ros
  anymal_description
  anymal_model_ros
  anymal_model
  anymal_msgs
  dodgeball_gazebo
  gazebo_ros
  param_io
  romo_std
  seabot_gazebo_plugin
  series_elastic_actuator_anydrive
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
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

########################
## Library definition ##
########################
add_library(${PROJECT_NAME}
  src/AnymalGazeboPlugin.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  PROGRAMS
    scripts/add_noise_to_description.py
    scripts/generate_rough_world.py
    scripts/generate_rough_world.sh
    scripts/generate_tilted_terrain.py
    scripts/generate_tilted_terrain.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
    scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES
    README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif()
