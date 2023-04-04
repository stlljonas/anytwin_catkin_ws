## Set up the CMake project.
cmake_minimum_required(VERSION 3.5.1)
project(anymal_ctrl_test_whole_body_control_msgs)

## Set the C++ standard.
set(CMAKE_CXX_STANDARD 14)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Find catkin dependencies for building this package.
find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder.
add_message_files(
  FILES
    MotionTrackingErrors.msg
)

## Generate added messages and services with any dependencies listed here.
generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects.
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
)
