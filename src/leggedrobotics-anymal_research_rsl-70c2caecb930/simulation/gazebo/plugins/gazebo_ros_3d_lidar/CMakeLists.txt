cmake_minimum_required(VERSION 3.5.1)
project(gazebo_ros_3d_lidar)
set(CMAKE_CXX_FLAGS "-std=c++0x -Wall -Wextra")

find_package(catkin
  REQUIRED
    diagnostic_updater
    gazebo_ros
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
    ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    diagnostic_updater
    gazebo_ros
  DEPENDS
    Boost
)

add_library(
  ${PROJECT_NAME}
  src/gazebo/GazeboRos3dLidar.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/gazebo/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
