# Project configuration
cmake_minimum_required (VERSION 2.8)

project(loco_ros_anymal)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  loco
  loco_ros
  loco_anymal
  tf
  anymal_description
  anymal_model_ros
  anymal_msgs
  visualization_msgs
  kindr_msgs
  kindr_ros
  std_utils
)

find_package(Eigen3 REQUIRED)

# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    loco
    loco_ros
    loco_anymal
    tf
    anymal_description
    anymal_model_ros
    anymal_msgs
    visualization_msgs
    kindr_msgs
    kindr_ros
    std_utils
)

add_definitions(--std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/visualization/LocomotionControllerRos.cpp
  src/visualization/GaitPatterns.cpp
  src/visualization/Ghost.cpp
  src/visualization/WholeBodyDynamics.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
