cmake_minimum_required(VERSION 3.5.1)
project(cosmo_node)

find_package(catkin REQUIRED COMPONENTS
  any_node
  cosmo_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    any_node
    cosmo_ros
)

###########
## Build ##
###########
add_definitions(-std=c++11 -fPIC -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##########
## Test ##
##########
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
