# Project configuration
cmake_minimum_required (VERSION 2.8)
project(loco_test)

add_definitions(-std=c++11)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  loco
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
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS
    loco
  DEPENDS
    kindr
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
)


#############
## Testing ##
#############

#catkin_add_gtest(${PROJECT_NAME}
#  src/test_main.cpp
#)
#if(TARGET ${PROJECT_NAME})
#  target_link_libraries(${PROJECT_NAME}
#    ${catkin_LIBRARIES}
#  )
#endif()

#find_package(cmake_code_coverage QUIET)
#if(cmake_code_coverage_FOUND)
#  add_gtest_coverage(TEST_BUILD_TARGETS ${PROJECT_NAME})
#endif(cmake_code_coverage_FOUND)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)