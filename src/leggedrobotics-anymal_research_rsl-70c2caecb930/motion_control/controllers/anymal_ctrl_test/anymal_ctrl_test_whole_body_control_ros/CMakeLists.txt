cmake_minimum_required(VERSION 3.5.1)
project(anymal_ctrl_test_whole_body_control_ros)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_HEADER_DEPENDENCIES
  anymal_ctrl_test_whole_body_control
)

set(CATKIN_PACKAGE_DEPENDENCIES
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
  any_state_estimator_msgs
  anymal_ctrl_test_whole_body_control_msgs
  eigen_conversions
  robot_control
  roscpp
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

# Build the core library.
add_library(${PROJECT_NAME}
  src/TestWholeBodyControllerRos.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Plugin ##
#############

# Build and install the plugin dependent library if robot_control_plugin exists
find_package(catkin QUIET COMPONENTS pluginlib ${CATKIN_PACKAGE_DEPENDENCIES})
if(${pluginlib_FOUND})
  add_library(${PROJECT_NAME}_plugin src/robot_control_plugin.cpp)
  target_include_directories(${PROJECT_NAME}_plugin PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    )
  target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )
  install(TARGETS ${PROJECT_NAME}_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
  install(FILES robot_control_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endif(${pluginlib_FOUND})

#############
## Clang ##
#############

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)