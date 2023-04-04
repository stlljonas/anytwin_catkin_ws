cmake_minimum_required(VERSION 3.5.1)
project(free_gait_stair_climbing)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  free_gait_core
  free_gait_ros
  kindr
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    free_gait_stair_climbing
  CATKIN_DEPENDS
    free_gait_core
    free_gait_ros
    kindr
  DEPENDS
    eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_lib
  src/MotionGenerator.cpp
  src/StairsGeometry.cpp
)

## Declare a cpp executable
add_executable(
  ${PROJECT_NAME}
  src/free_gait_stair_climbing_node.cpp
  src/StairClimbing.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}_lib-test
    test/test_free_gait_stair_climbing.cpp
    test/StairsGeometryTest.cpp
  )

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_rostest_coverage(${PROJECT_NAME}_lib-test)
  endif(cmake_code_coverage_FOUND)
endif()

if(TARGET ${PROJECT_NAME}_lib-test)
  target_link_libraries(${PROJECT_NAME}_lib-test ${PROJECT_NAME}_lib)
endif()

install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES actions.yaml collections.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
