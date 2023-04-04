cmake_minimum_required(VERSION 3.5.1)
project(depth_sensor_tilt_calibration)

add_definitions(-std=c++11 -Wall -Werror -Wno-deprecated-declarations)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    kindr_ros
    pcl_ros
    roscpp
    sensor_msgs
    tf
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
  CATKIN_DEPENDS
    geometry_msgs
    kindr_ros
    pcl_ros
    roscpp
    sensor_msgs
    tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${kindr_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(depth_sensor_tilt_calibration
#   src/${PROJECT_NAME}/depth_sensor_tilt_calibration.cpp
# )

## Declare a cpp executable
add_executable(
	${PROJECT_NAME}
	src/depth_sensor_tilt_calibration_node.cpp
	src/DepthSensorTiltCalibration.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(depth_sensor_tilt_calibration_node depth_sensor_tilt_calibration_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
      ${kindr_INCLUDE_DIRS}
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
  )
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