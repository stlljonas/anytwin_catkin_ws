cmake_minimum_required(VERSION 3.5.1)
project(free_gait_anymal_common)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  any_measurements
  any_measurements_ros
  free_gait_core
  free_gait_ros
  geometry_utils
  geometry_utils_ros
  grid_map_core
  grid_map_ros
  anymal_model
  anymal_model_ros
)

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
      test/include
    LIBRARIES
      ${PROJECT_NAME}
    CATKIN_DEPENDS
      any_measurements
      any_measurements_ros
      free_gait_core
      free_gait_ros
      geometry_utils
      geometry_utils_ros
      grid_map_core
      grid_map_ros
      anymal_model
      anymal_model_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  test/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/AdapterAnymal.cpp
  src/AdapterAnymalPreview.cpp
  src/AdapterRosInterfaceAnymal.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}-test
    test/free_gait_anymal_common.test
    test/test_free_gait_anymal_common.cpp
    test/StateRosPublisherTest.cpp
    test/PoseOptimizationSqpAnymalTest.cpp
  )

  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()

endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ test/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_description.xml plugin_ros_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
