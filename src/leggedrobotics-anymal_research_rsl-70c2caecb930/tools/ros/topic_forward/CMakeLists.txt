cmake_minimum_required(VERSION 3.5.1)
project(topic_forward)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    param_io
    roscpp
    tf
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    param_io
    roscpp
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  tf_forward
  src/tf_forward_node.cpp
)
target_link_libraries(
  tf_forward
  ${catkin_LIBRARIES}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS tf_forward
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
