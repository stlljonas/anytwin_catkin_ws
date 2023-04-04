cmake_minimum_required(VERSION 3.5.1)
project(anymal_chimera)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
