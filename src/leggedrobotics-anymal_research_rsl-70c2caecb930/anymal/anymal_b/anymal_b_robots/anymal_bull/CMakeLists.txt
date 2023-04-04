cmake_minimum_required(VERSION 3.5.1)
project(anymal_bull)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    config
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
