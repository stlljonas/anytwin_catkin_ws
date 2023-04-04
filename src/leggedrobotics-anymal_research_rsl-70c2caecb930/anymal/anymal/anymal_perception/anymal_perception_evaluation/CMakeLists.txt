cmake_minimum_required(VERSION 3.5.1)
project(anymal_perception_evaluation)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
