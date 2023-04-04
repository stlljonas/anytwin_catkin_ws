# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(motion_generation_msgs)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)

# Declare the message and service files to be built
add_service_files(
  FILES
    GetAvailableGaits.srv
    SwitchGait.srv
    SetStride.srv
    GetStride.srv
)
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Generate a package
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)