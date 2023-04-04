cmake_minimum_required(VERSION 3.5.1)
project(rpsm_lpc)

add_definitions(-std=c++11 -Wall -Werror)


find_package(catkin REQUIRED COMPONENTS
  rpsm_master
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    rpsm_lpc
  CATKIN_DEPENDS
    rpsm_master
  DEPENDS
    Boost
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_library(rpsm_lpc
  src/RpsmNode.cpp
)

add_dependencies(rpsm_lpc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(rpsm_lpc_node
  src/rpsm_lpc_node.cpp
)

add_dependencies(rpsm_lpc_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rpsm_lpc_node
  rpsm_lpc
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS rpsm_lpc rpsm_lpc_node
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
