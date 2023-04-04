cmake_minimum_required(VERSION 3.5.1)
project(anymal_b)

find_package(catkin)

catkin_package()

install(
  DIRECTORY
    config
    launch
    urdf
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
