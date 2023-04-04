cmake_minimum_required(VERSION 3.5.1)
project(icp_tools_global_mapping)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  icp_tools_common
  kindr
  kindr_ros
  libpointmatcher
  message_logger
  param_io
  pointmatcher_ros
  roscpp
  slam_point_cloud_mapping
)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)
find_package(cmake_clang_tools QUIET)

# Catkin package macro
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${kindr_INCLUDE_DIRS}
)

########################
## Library definition ##
########################
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/SlamNode.cpp
  src/${PROJECT_NAME}/PointCloudGlobalMapBuilder.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

################
## Executable ##
################
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
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
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )

  include_directories(
    include
    SYSTEM
      ${EIGEN3_INCLUDE_DIRS}
      ${libpointmatcher_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
endif()

#################
## Clang_tools ##
#################
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif()

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