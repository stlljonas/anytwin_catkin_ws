cmake_minimum_required(VERSION 3.5.1)
project(rpsm_apc)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  any_msgs
  any_node
  std_srvs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    rpsm_apc
  CATKIN_DEPENDS
    any_msgs
    any_node
    std_srvs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(rpsm_apc
  src/RpsmNode.cpp
)

add_dependencies(rpsm_apc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(rpsm_apc_node
  src/rpsm_apc_node.cpp
)

add_dependencies(rpsm_apc_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rpsm_apc_node
   rpsm_apc
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS rpsm_apc rpsm_apc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
