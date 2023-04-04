# Project configuration
cmake_minimum_required (VERSION 2.8)

project(loco_ros)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  any_node
  loco
  tf
  visualization_msgs
  rocoma_msgs
  kindr_msgs
  kindr_ros
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
    any_node
    loco
    tf
    visualization_msgs
    rocoma_msgs
    kindr_msgs
    kindr_ros
)

add_definitions(--std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} 
  src/loco_ros.cpp
  src/SteeringControllerRos.cpp
  src/visualization/SupportPolygon.cpp
  src/visualization/Arm.cpp
  src/visualization/Torso.cpp
  src/visualization/FrictionPyramid.cpp
  src/visualization/FrictionCone.cpp
  src/visualization/Arrow.cpp
  src/visualization/Point.cpp
  src/visualization/SwingTrajectoryGeneratorSpline.cpp
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

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
