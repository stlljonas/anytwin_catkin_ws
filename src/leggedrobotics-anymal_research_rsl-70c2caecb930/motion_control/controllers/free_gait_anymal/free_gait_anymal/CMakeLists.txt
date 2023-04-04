cmake_minimum_required(VERSION 3.5.1)
project(free_gait_anymal)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  free_gait_anymal_common
  free_gait_action_loader
  locomotion_planner
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.launch"
)
