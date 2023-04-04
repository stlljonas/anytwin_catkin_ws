cmake_minimum_required(VERSION 3.5.1)
project(locomotion_planner)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-DMELO_FUNCTION_PRINTS ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_logger
  free_gait_core
  free_gait_msgs
  free_gait_ros
  geometry_msgs
  kindr
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_sdf
  loco
  numopt_common
  numopt_osqp
  numopt_sqp
  robot_utils
  visualization_msgs
  actionlib
  locomotion_planner_msgs
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
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    message_logger
    free_gait_core
    free_gait_msgs
    free_gait_ros
    geometry_msgs
    kindr
    grid_map_core
    grid_map_ros
    grid_map_msgs
    grid_map_sdf
    loco
    numopt_common
	numopt_osqp
  	numopt_sqp
  	robot_utils
  	visualization_msgs
	actionlib
	locomotion_planner_msgs
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/LocomotionPlanner.cpp
  src/LocomotionPlannerRos.cpp
  src/common/type_prints.cpp
  src/common/Parameters.cpp
  src/common/PlannerModuleBase.cpp
  src/common/geometry.cpp
  src/common/ElevationMapUser.cpp
  src/common/PlanningData.cpp
  src/gait_pattern/GaitPatternBase.cpp
  src/gait_pattern/GaitPatternCrawling.cpp
  src/gait_pattern/GaitPatternLongitudinalCrawling.cpp
  src/foothold_optimizer/FootholdOptimizerBase.cpp
  src/foothold_optimizer/FootholdOptimizerSameHeight.cpp
  src/foothold_optimizer/FootholdOptimizerPlaneFit.cpp
  src/foothold_optimizer/FootholdOptimizerElevationMap.cpp
  src/foothold_optimizer/FootholdOptimizerElevationMapAndKinematics.cpp
  src/footstep_planner/FootstepPlannerBase.cpp
  src/footstep_planner/OneStepFootstepPlanner.cpp
  src/swing_trajectory_planner/SwingTrajectoryPlannerBase.cpp
  src/swing_trajectory_planner/SwingHeightPlanner.cpp
  src/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryPlanner.cpp
  src/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryParameterization.cpp
  src/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryObjectiveFunction.cpp
  src/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryFunctionConstraints.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_locomotion_planner_core.cpp
  test/OneStepFootstepPlannerTest.cpp
  test/GaitPatternTest.cpp
)

find_package(cmake_code_coverage QUIET)
if(cmake_code_coverage_FOUND)
	add_gtest_coverage(TEST_BUILD_TARGETS ${PROJECT_NAME}-test)
endif(cmake_code_coverage_FOUND)


if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
  )
endif()

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY 
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
