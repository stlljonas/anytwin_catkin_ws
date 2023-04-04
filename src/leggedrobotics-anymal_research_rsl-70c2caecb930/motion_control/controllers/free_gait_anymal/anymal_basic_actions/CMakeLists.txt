# Project configuration
cmake_minimum_required (VERSION 3.5.1)
project(anymal_basic_actions)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    any_msgs
    anymal_msgs
    free_gait_action_loader
    rocoma_msgs
    std_srvs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    any_msgs
    anymal_msgs
    free_gait_action_loader
    rocoma_msgs
    std_srvs
)

install(DIRECTORY
    actions
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
    actions.yaml
    collections.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
