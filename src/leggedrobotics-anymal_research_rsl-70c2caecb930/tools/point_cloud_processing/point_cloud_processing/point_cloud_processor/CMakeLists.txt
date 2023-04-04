cmake_minimum_required(VERSION 3.5.1)
project(point_cloud_processor)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  geometry_msgs
  message_logger
  nodelet
  param_io
  point_cloud_self_filter
  pointmatcher_ros
  roscpp
  std_utils
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
  tinyxml_tools
  urdf
  yaml_tools
)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(cmake_clang_tools QUIET)
find_package(libpointmatcher REQUIRED)

# Catkin package macro
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  SYSTEM # Don't hickup on warnings from included files wih -Werror
    ${EIGEN3_INCLUDE_DIR}
    ${libpointmatcher_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

########################
## Library definition ##
########################
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/CloudStream.cpp
  src/${PROJECT_NAME}/PointCloudProcessor.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
)

if (CMAKE_BUILD_TYPE EQUAL "RELEASE")
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported)
  if(ipo_supported)
      set_property(TARGET ${PROJECT_NAME}_core PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

##########################
##  Nodes and nodelets  ##
##########################
# Node
add_executable(${PROJECT_NAME}_node
  src/point_cloud_processor_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Nodelet
add_library(${PROJECT_NAME}_nodelet
  src/PointCloudProcessorNodelet.cpp
)

add_dependencies(${PROJECT_NAME}_nodelet
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
  )

  include_directories(
    include
    SYSTEM
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${libpointmatcher_INCLUDE_DIRS}
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

#################
## Clang_tools ##
#################
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
      ${PROJECT_NAME}_node
      ${PROJECT_NAME}_nodelet
    SOURCE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)

###################
## Code_coverage ##
###################
find_package(cmake_code_coverage QUIET)
if(cmake_code_coverage_FOUND)
  add_gtest_coverage(
    TEST_BUILD_TARGETS
      test_${PROJECT_NAME}
  )
endif(cmake_code_coverage_FOUND)
