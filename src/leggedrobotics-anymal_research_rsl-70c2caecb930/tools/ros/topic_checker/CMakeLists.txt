cmake_minimum_required(VERSION 3.5.1)
project(topic_checker)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    any_msgs
    geometry_msgs
    param_io
    roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    any_msgs
    geometry_msgs
    param_io
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  pose_stamped_checker
  src/pose_stamped_checker_node.cpp
)
target_link_libraries(
  pose_stamped_checker
  ${catkin_LIBRARIES}
)

add_executable(
  twist_stamped_checker
  src/twist_stamped_checker_node.cpp
)
target_link_libraries(
  twist_stamped_checker
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS pose_stamped_checker twist_stamped_checker
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
