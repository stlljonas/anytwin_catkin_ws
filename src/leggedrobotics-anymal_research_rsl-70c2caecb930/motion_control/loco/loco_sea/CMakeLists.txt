cmake_minimum_required(VERSION 3.5.1)
project(loco_sea)

find_package(catkin REQUIRED COMPONENTS
  loco
  series_elastic_actuator
  std_utils
)

add_definitions(-std=c++11)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    loco
    series_elastic_actuator
    std_utils
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
