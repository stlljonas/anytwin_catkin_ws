cmake_minimum_required(VERSION 3.5.1)
project(bageditor)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rosbag
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rosbag
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
