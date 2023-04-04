cmake_minimum_required(VERSION 3.5.1)
project(actuated_lidar_description)

add_definitions(-std=c+11)

find_package(catkin REQUIRED
  COMPONENTS
    dynamixel_description
)

catkin_package(
  CATKIN_DEPENDS
    dynamixel_description
)

install(DIRECTORY launch mesh urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
