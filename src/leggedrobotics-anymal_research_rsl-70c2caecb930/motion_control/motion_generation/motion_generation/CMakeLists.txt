# Project configuration
cmake_minimum_required (VERSION 2.8)
project(motion_generation)

add_definitions(-std=c++11)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
    anymal_description
    basic_filters
    robot_utils
    message_logger
    tinyxml_tools
    loco
    std_utils
    signal_logger
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
    robot_utils
    message_logger
    tinyxml_tools
    loco
    std_utils
    signal_logger
 DEPENDS kindr
)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/ContactSchedulePeriodic.cpp
  src/ContactSchedulePeriodicSwitch.cpp
  src/ContactScheduleStrideControl.cpp
  src/ContactScheduleZmp.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  # Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/GaitDescriptionTest.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()
endif()

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
        PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
