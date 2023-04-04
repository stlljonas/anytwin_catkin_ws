cmake_minimum_required (VERSION 2.8)
project(romo_measurements_test)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
	romo_rbdl
  romo_measurements
  loco
	loco_test
  romo
)

# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
		romo_rbdl
		romo_measurements
    loco
		loco_test
		romo
)

add_definitions(--std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
