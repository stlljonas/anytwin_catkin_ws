cmake_minimum_required (VERSION 2.8)
project(romo_measurements)

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
	romo
)

# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    loco
		romo
  DEPENDS
    EIGEN3
)

add_definitions(--std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
