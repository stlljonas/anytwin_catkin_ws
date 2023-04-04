cmake_minimum_required(VERSION 3.1.3)
project(free_gait_anymal_interactive_test)

set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  free_gait_anymal_common
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
    CATKIN_DEPENDS
  free_gait_anymal_common
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
      interactive_markers
      free_gait_anymal_common
      free_gait_core
      free_gait_ros
      rostest
  )
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

#  add_rostest_gtest(${PROJECT_NAME}
#    test/free_gait_anymal_interactive_test.test
#    test/test_free_gait_anymal_interactive.cpp
#    test/PreviewTest.cpp
#    test/PoseOptimizationSqpAnymalInteractiveTest.cpp
#  )

#  find_package(cmake_code_coverage QUIET)
#  if(cmake_code_coverage_FOUND)
#    add_rostest_coverage(${PROJECT_NAME})
#  endif(cmake_code_coverage_FOUND)

endif()

if(TARGET ${PROJECT_NAME})
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()
