cmake_minimum_required(VERSION 3.5.1)
project(topic_mux)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    any_msgs
    geometry_msgs
    roscpp
    std_srvs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    any_msgs
    geometry_msgs
    roscpp
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Pose
add_executable(
  pose_stamped_mux
  src/pose_stamped_mux_node.cpp
)
add_dependencies(
  pose_stamped_mux
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  pose_stamped_mux
  ${catkin_LIBRARIES}
)

# Twist
add_executable(
  twist_stamped_mux
  src/twist_stamped_mux_node.cpp
)
add_dependencies(
  twist_stamped_mux
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  twist_stamped_mux
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS pose_stamped_mux twist_stamped_mux
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.launch"
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.conf"
  PATTERN "*.yaml"
)
