# Project configuration
cmake_minimum_required (VERSION 3.1.3)
project(motion_generation_loco)

# Compile as C++14, supported in ROS Melodic and newer.
set(CMAKE_CXX_STANDARD 14)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
    anymal_description
    loco
    robot_utils
    motion_generation
    motion_generation_utils
    zmp_optimizer
    tinyxml_tools
    numopt_quadprog
    numopt_common
    std_utils
    terrain_sensing
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)


# Declare this project as a catkin package
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
     loco
     robot_utils
     motion_generation
     motion_generation_utils
     zmp_optimizer
     tinyxml_tools
     numopt_quadprog
     numopt_common
     std_utils
     terrain_sensing
 DEPENDS kindr
)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  # Planner
  src/planner/TorsoPlannerZmp.cpp

  # Torso control
  src/torso_control/ComSupportControlZmp.cpp
  src/torso_control/TorsoControlZmp.cpp

  # Foothold generator
  src/foothold_generation/FootholdGeneratorInvertedPendulumMotionGen.cpp
  src/foothold_generation/FootholdGeneratorInvertedPendulumDynamicWalk.cpp
  src/foothold_generation/FootholdGeneratorOptimizedQPBase.cpp
  src/foothold_generation/FootholdGeneratorOptimizedConstraint.cpp
  src/foothold_generation/FootholdGeneratorOptimizedInvPend.cpp
  src/foothold_generation/FootholdGeneratorInvertedPendulumPathRegularizer.cpp

  # Mission control
  src/mission_control/MissionControlZmp.cpp
  
  # Swing Trajectory
  src/swing_trajectory_generation/SwingTrajectoryGeneratorSplineOpt.cpp
  
  # Terrain Perception
  src/terrain_perception/TerrainPerceptionPredictiveFreePlane
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

############
# Testing ##
############
catkin_add_gtest(test_${PROJECT_NAME}
   test/test_main.cpp
)
if(TARGET test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
   )
endif()

find_package(cmake_code_coverage QUIET)
if(cmake_code_coverage_FOUND)
  add_gtest_coverage()
endif(cmake_code_coverage_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/loco/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/loco
        PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
