cmake_minimum_required(VERSION 3.5.1)
project(dynamixel_description)

add_definitions(-std=c+11)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY mesh urdf_examples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)