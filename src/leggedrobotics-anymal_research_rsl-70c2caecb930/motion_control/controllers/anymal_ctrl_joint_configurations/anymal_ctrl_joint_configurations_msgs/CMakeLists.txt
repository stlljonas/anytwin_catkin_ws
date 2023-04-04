## Set up the CMake project.
cmake_minimum_required(VERSION 3.5.1)
project(anymal_ctrl_joint_configurations_msgs)

## Set the C++ standard.
set(CMAKE_CXX_STANDARD 14)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Find catkin dependencies for building this package.
find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    message_generation
    std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder.
#add_message_files(
#  FILES
#    Message1.msg
#    Message2.msg
#)

## Generate services in the 'srv' folder.
#add_service_files(
#  FILES
#    Service1.srv
#    Service2.srv
#)

## Generate actions in the 'action' folder.
add_action_files(
  FILES
    JointConfiguration.action
)

## Generate added messages and services with any dependencies listed here.
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects.
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
    std_msgs
)
