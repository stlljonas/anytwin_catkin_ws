cmake_minimum_required(VERSION 3.5.1)
project(rpsm_master)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  any_msgs
  any_node
  roscpp
  rpsm_msgs
  sensor_msgs
  serial
  std_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS
    include
    ${serial_INCLUDE_DIRS}
  LIBRARIES
    rpsm_master
  CATKIN_DEPENDS
    actionlib
    any_msgs
    any_node
    roscpp
    rpsm_msgs
    sensor_msgs
    serial
    std_msgs
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

add_library(rpsm_master
  src/RpsmMaster.cpp
)

target_link_libraries(rpsm_master
  ${catkin_LIBRARIES}
)

add_dependencies(rpsm_master
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_library(rpsm_mainboard
  src/RpsmMainboard.cpp
  src/FileManager.cpp)

target_link_libraries(rpsm_mainboard
  rpsm_master
  ${catkin_LIBRARIES}
)

add_dependencies(rpsm_mainboard
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(rpsm_mainboard_node
  src/rpsm_mainboard_node.cpp
)

add_dependencies(rpsm_mainboard_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rpsm_mainboard_node
  ${catkin_LIBRARIES}
  rpsm_mainboard
)

#############
## Install ##
#############

install(TARGETS rpsm_master rpsm_mainboard rpsm_mainboard_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
