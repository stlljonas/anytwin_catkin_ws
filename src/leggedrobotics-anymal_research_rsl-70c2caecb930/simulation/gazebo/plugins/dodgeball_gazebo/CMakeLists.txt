cmake_minimum_required(VERSION 3.5.1)
project(dodgeball_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_msgs
    gazebo_ros
    geometry_msgs
    kindr
    visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

catkin_package(
  INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIRS}
  LIBRARIES
    ${OPENGL_LIBRARIES}
  CATKIN_DEPENDS
    gazebo_msgs
    gazebo_ros
    geometry_msgs
    kindr
    visualization_msgs
  DEPENDS
    GLUT
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${kindr_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
)

add_executable(dodgeball_node
  src/dodgeball_node.cpp
)

target_link_libraries(dodgeball_node
  ${catkin_LIBRARIES}
  ${GLUT_LIBRARY}
  ${OPENGL_LIBRARIES}
)

install(TARGETS dodgeball_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
