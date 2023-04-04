cmake_minimum_required(VERSION 3.5.1)
project(anymal_wide_angle_cameras)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
