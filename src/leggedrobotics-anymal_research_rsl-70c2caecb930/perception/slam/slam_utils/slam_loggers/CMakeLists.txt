cmake_minimum_required(VERSION 3.5.1)
project(slam_loggers)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  gazebo_msgs
  geometry_msgs
  message_logger
  nav_msgs
  roscpp
  slam_common
  slam_common_ros
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(cmake_clang_tools QUIET)

# Catkin package macro
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

########################
## Library definition ##
########################
add_library(${PROJECT_NAME}
  src/BaseLogger.cpp
  src/GazeboGroundTruthPosePublisher.cpp
  src/KittiLogger.cpp
  src/PoseLoggerRos.cpp
  src/TumLogger.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

###########
## Nodes ##
###########
add_executable(gazebo_ground_truth_pose_publisher
  src/gazebo_ground_truth_pose_publisher_node.cpp
)
target_link_libraries(gazebo_ground_truth_pose_publisher
  ${PROJECT_NAME}
)

add_executable(pose_logger
  src/pose_logger_node.cpp
)
target_link_libraries(pose_logger
  ${PROJECT_NAME}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
    gazebo_ground_truth_pose_publisher
    pose_logger
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES
    README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
  )

  include_directories(
    include
    SYSTEM
      ${catkin_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
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