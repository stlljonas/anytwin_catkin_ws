cmake_minimum_required(VERSION 3.5.1)
project(kindr_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
      kindr
)

find_package(gazebo REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS kindr
  DEPENDS gazebo
)

# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
    test/ConventionTest.cpp 
    test/test_main.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${GAZEBO_LIBRARIES})
endif()