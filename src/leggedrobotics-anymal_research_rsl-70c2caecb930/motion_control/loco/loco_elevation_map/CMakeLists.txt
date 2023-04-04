# Project configuration
cmake_minimum_required (VERSION 2.8)

project(loco_elevation_map)


# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  loco
  grid_map_core
  any_worker
  tinyxml_tools
)

# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    loco
    grid_map_core
    any_worker
    tinyxml_tools
)

add_definitions(--std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  # common
  src/TerrainModelElevationMap.cpp
  
  # foothold planning
  src/FootholdGeneratorInvertedPendulumElevationMap.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
