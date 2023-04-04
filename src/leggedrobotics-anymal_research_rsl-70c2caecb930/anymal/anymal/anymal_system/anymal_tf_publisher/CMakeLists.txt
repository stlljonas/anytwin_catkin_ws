cmake_minimum_required(VERSION 3.5.1)
project(anymal_tf_publisher)

# Activate C++11.
set(CMAKE_CXX_FLAGS "-std=c++11")

set(PACKAGE_DEPENDENCIES
    any_node
    anymal_description
    anymal_model_ros
    anymal_msgs
    tf
    robot_state_publisher
    cosmo_ros
    any_measurements_ros
    series_elastic_actuator_ros
)

find_package(catkin
    REQUIRED COMPONENTS
      ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES anymal_tf_publisher
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
#  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(anymal_tf_publisher
  src/AnymalTfPublisher.cpp
)

add_dependencies(anymal_tf_publisher ${catkin_EXPORTED_TARGETS})

target_link_libraries(anymal_tf_publisher
   ${catkin_LIBRARIES}
)

add_executable(anymal_tf_publisher_node
  src/anymal_tf_publisher_node.cpp
)

add_dependencies(anymal_tf_publisher_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(anymal_tf_publisher_node
   anymal_tf_publisher
   ${catkin_LIBRARIES}
)

install(TARGETS anymal_tf_publisher anymal_tf_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.launch"
)
