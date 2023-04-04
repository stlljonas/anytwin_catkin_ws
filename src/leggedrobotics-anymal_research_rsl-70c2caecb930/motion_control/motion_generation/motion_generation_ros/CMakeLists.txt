# Project configuration
cmake_minimum_required (VERSION 2.8)
project(motion_generation_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    motion_generation_loco
    loco_ros
    motion_generation
    swing_trajectory_generation
    zmp_optimizer
    terrain_sensing
    visualization_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motion_generation_ros
  CATKIN_DEPENDS
    motion_generation_loco
    loco_ros
    motion_generation
    swing_trajectory_generation
    zmp_optimizer
    terrain_sensing
    visualization_msgs
    geometry_msgs
#  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)

add_definitions(--std=c++11)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_library(motion_generation_ros SHARED
   src/SupportPolygonSequence.cpp
   src/MotionPlanVisualizer.cpp
   src/FootholdPlanVisualizer.cpp
   src/FinalComBox.cpp
   src/SwingTrajectoryGeneratorSplineOptimized.cpp
   src/SwingTrajectoryGeneratorSplineOptVisualizer.cpp
   src/StateCheckerPublisher.cpp
)

add_dependencies(motion_generation_ros ${catkin_EXPORTED_TARGETS})

target_link_libraries(motion_generation_ros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
        PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
