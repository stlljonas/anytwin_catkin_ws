# Project configuration
cmake_minimum_required (VERSION 2.8)

project(loco_elevation_map_ros)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  loco
  loco_ros
  loco_ros_anymal
  loco_elevation_map
  tf
  visualization_msgs
  kindr_msgs
  kindr_ros
  grid_map_core
  grid_map_ros
  grid_map_msgs
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
    loco_ros_anymal
    loco_elevation_map
    tf
    visualization_msgs
    kindr_msgs
    kindr_ros
    grid_map_core
    grid_map_ros
    grid_map_msgs
)

add_definitions(--std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} 
  src/ElevationMapSubscriber.cpp
  src/LocomotionControllerElevationMapRos.cpp
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
