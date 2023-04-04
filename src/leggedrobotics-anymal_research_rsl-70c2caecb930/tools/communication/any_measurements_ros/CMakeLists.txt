cmake_minimum_required(VERSION 3.5.1)
project(any_measurements_ros)

find_package(catkin REQUIRED COMPONENTS
  any_measurements
  kindr_ros
  sensor_msgs
  geometry_msgs
  any_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    any_measurements
    kindr_ros
    sensor_msgs
    geometry_msgs
    any_msgs
)

###########
## Build ##
###########
add_definitions(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#add_library(${PROJECT_NAME}
#
#)


##########
## Test ##
##########
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    ${PROJECT_NAME}_test
    test/${PROJECT_NAME}_test.cpp
    test/ConversionTraitsTest.cpp
  )
endif()

if(TARGET ${PROJECT_NAME}_test)
  target_link_libraries(${PROJECT_NAME}_test
    ${catkin_LIBRARIES}
  )
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
