cmake_minimum_required(VERSION 3.5.1)
project(joy_state_estimation)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  any_msgs
  anymal_msgs
  joy_manager
  std_srvs
)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Catkin package macro
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Specify locations of header files
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

########################
## Library definition ##
########################
add_library(${PROJECT_NAME}
    src/JoyStateEstimation.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES
    joy_state_estimation.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
  )

  include_directories(
    include
    SYSTEM
      ${catkin_INCLUDE_DIRS}
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

###################
## Code_coverage ##
###################
find_package(cmake_code_coverage QUIET)
if(cmake_code_coverage_FOUND)
  add_gtest_coverage(
    TEST_BUILD_TARGETS
      test_${PROJECT_NAME}
  )
endif(cmake_code_coverage_FOUND)