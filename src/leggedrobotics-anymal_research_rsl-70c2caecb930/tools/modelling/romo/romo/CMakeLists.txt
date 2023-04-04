# Project configuration
cmake_minimum_required (VERSION 2.8)
project(romo)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  basic_filters
  robot_utils
  std_utils
  collisions_geometry
  collisions_urdf
  collisions
)

find_package(Eigen3 REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS
    basic_filters
    robot_utils
    std_utils
    collisions_geometry
    collisions_urdf
    collisions
  DEPENDS
    kindr
)

###########
## Build ##
###########
add_definitions(-std=c++11 -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
)


#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

# Build unit tests
#catkin_add_gtest(test_romo 
#)
#add_dependencies(test_romo ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_romo
#)
