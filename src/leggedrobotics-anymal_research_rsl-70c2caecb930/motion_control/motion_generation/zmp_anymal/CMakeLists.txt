# Project configuration
cmake_minimum_required (VERSION 2.8)
project(zmp_anymal)

add_definitions(-std=c++11)

set(PACKAGE_DEPENDENCIES
    anymal_description
    anymal_model
    anymal_ctrl_dynamic_gaits
    cosmo_ros
    kindr_ros
    joy_anymal_lowlevel_controller
    joy_state_estimation
    loco_anymal
    loco_ros_anymal
    numopt_quadprog
    numopt_common
    motion_generation
    motion_generation_loco
    motion_generation_msgs
    motion_generation_ros
    motion_generation_utils
    message_logger
    robot_utils
    tinyxml_tools
    rqt_gait_switcher
    rqt_highlevel_controlmanager
    rqt_loco_visualizer
    loco
    param_io
    parameter_handler
    parameter_handler_ros
    interactive_markers
    anymal_tf_publisher
    zmp_optimizer
    joy_manager
)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
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
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS kindr
)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/ContactScheduleZmpTest.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

# Interactive test added as an executable
add_executable(${PROJECT_NAME}_interactive_test_node
  test/zmp_interactive_test.cpp
)
add_dependencies(${PROJECT_NAME}_interactive_test_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_interactive_test_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)