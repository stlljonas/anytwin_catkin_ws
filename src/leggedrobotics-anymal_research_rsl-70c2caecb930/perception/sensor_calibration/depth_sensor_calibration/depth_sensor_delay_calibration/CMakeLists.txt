cmake_minimum_required(VERSION 3.5.1)
project(depth_sensor_delay_calibration)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED
  COMPONENTS
    any_msgs
    param_io
    pcl_conversions
    pointmatcher_ros
    roscpp
    sensor_msgs
    tf
    tf2_ros
)
find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${Eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS
    any_msgs
    param_io
    pointmatcher_ros
    roscpp
    sensor_msgs
    tf
    tf2_ros
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${libpointmatcher_INCLUDE_DIRS}
    ${kindr_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/DepthSensorDelayCalibration.cpp
  src/main.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED)

  include_directories(
    include
    SYSTEM
      ${catkin_INCLUDE_DIRS}
      ${Eigen3_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
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