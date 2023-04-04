cmake_minimum_required(VERSION 3.5.1)
project(pose_tf_publisher)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    tf
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  pose_tf_publisher
  src/pose_tf_publisher_node.cpp
)
target_link_libraries(
  pose_tf_publisher
  ${catkin_LIBRARIES}
)

add_executable(
  pose_with_covariance_tf_publisher
  src/pose_with_covariance_tf_publisher_node.cpp
)
target_link_libraries(
  pose_with_covariance_tf_publisher
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS pose_tf_publisher pose_with_covariance_tf_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
