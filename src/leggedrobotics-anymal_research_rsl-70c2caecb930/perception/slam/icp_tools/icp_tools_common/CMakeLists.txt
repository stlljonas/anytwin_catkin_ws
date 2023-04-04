cmake_minimum_required(VERSION 3.5.1)
project(icp_tools_common)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  any_msgs
  geometry_msgs
  slam_common_msgs
  kindr
  kindr_ros
  libpointmatcher
  message_logger
  param_io
  pointmatcher_ros
  roscpp
  sensor_msgs
  slam_common
  slam_common_ros
  std_msgs
  std_srvs
  stopwatch
  tf
  tf_conversions
  tf2
)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)
find_package(Eigen3 REQUIRED)
find_package(cmake_clang_tools QUIET)

# Catkin package macro
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${kindr_INCLUDE_DIRS}
)

#############
## Install ##
#############
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  include_directories(
    include
    SYSTEM
      ${EIGEN3_INCLUDE_DIRS}
      ${libpointmatcher_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
  )  

  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    WORKING_DIRECTORY 
      ${PROJECT_SOURCE_DIR}/test
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
endif()

#################
## Clang_tools ##
#################
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif()

###################
## Code_coverage ##
###################
find_package(cmake_code_coverage QUIET)
if(cmake_code_coverage_FOUND)
  add_gtest_coverage(
    TEST_BUILD_TARGETS
      test_${PROJECT_NAME}
  )
endif(cmake_code_coverage_FOUND)