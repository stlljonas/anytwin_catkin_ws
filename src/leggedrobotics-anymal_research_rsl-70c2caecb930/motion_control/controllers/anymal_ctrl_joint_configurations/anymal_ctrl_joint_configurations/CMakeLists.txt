## Set up the CMake project.
cmake_minimum_required(VERSION 3.5.1)
project(anymal_ctrl_joint_configurations)

## Set the C++ standard.
set(CMAKE_CXX_STANDARD 14)

## Enable all compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Create a list of catkin dependencies.
set(CATKIN_PACKAGE_DEPENDENCIES
  anymal_description
  anymal_motion_control
  message_logger
  pluginlib
  robot_control
  tinyxml_tools
)

## Find catkin dependencies for building and execution.
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

## Find system dependencies providing a CMake module.
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

###########
## Build ##
###########

## Declare a C++ library.
add_library(${PROJECT_NAME}
  src/JointConfigurationsController.cpp
  src/MotionGeneration.cpp
)

## Add CMake target dependencies of the library.
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

## Specify additional locations of header files of the library.
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Specify libraries to link executable targets against.
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation.
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation.
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

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  add_default_clang_tooling(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin)
endif (cmake_clang_tools_FOUND)
