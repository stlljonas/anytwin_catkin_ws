# Project configuration
cmake_minimum_required (VERSION 2.8)
project(anymal_free_gait_action_loader_adapter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  free_gait_msgs
  anymal_msgs
)

catkin_package(
  #INCLUDE_DIRS
  #LIBRARIES
  #CATKIN_DEPENDS
  #DEPENDS
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#############
## Install ##
#############

install(PROGRAMS
  bin/anymal_free_gait_action_loader_adapter/action_loader_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING 
  PATTERN "*.launch"
)
