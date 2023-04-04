# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(actuated_lidar_msgs)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
    MotionGoalReached.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)

# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

