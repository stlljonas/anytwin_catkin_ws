cmake_minimum_required(VERSION 3.5.1)
project(anymal_c_actions)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    actions
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  FILES
    actions.yaml
    collections.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
