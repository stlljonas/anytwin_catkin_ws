cmake_minimum_required(VERSION 3.5.1)
project(anymal_velodyne)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY calib launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
