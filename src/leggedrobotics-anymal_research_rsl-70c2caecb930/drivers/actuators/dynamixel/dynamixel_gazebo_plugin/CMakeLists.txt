cmake_minimum_required(VERSION 3.5.1)
project(dynamixel_gazebo_plugin)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_updater
    dynamixel_controllers
    dynamixel_msgs
    dynamixel_ros_msgs
    gazebo_plugins
    roscpp
    sensor_msgs
    std_msgs
    urdf
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

#catkin_python_setup()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
    ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    diagnostic_updater
    dynamixel_controllers
    dynamixel_msgs
    dynamixel_ros_msgs
    gazebo_plugins
    roscpp
    sensor_msgs
    std_msgs
    urdf
  DEPENDS
    Boost
)

add_definitions(-fPIC) # what is this for?

add_library(${PROJECT_NAME}
  src/gazebo_ros_dynamixel_motor.cpp
)
add_dependencies(${PROJECT_NAME}
  dynamixel_ros_msgs_generate_messages_cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
