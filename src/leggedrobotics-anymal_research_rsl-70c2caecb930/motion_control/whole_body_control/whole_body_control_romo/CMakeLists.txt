# Project configuration
cmake_minimum_required (VERSION 2.8)
project(whole_body_control_romo)

add_definitions(-std=c++11)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
    loco
    numopt_quadprog
    hierarchical_optimization
    whole_body_control
    romo
    tinyxml_tools
    basic_controllers
    parameter_handler
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)


# Declare this project as a catkin package
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
    loco
    numopt_quadprog
    hierarchical_optimization
    whole_body_control
    romo
    tinyxml_tools
    basic_controllers
    parameter_handler
 DEPENDS kindr
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${kindr_INCLUDE_DIRS})


#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
        PATTERN ".svn" EXCLUDE
)
