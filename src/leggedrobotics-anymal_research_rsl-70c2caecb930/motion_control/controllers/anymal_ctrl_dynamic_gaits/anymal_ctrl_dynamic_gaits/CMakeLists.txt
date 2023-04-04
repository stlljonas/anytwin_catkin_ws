# Project configuration
cmake_minimum_required (VERSION 2.8)
project(anymal_ctrl_dynamic_gaits)

# List of dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  anymal_description
  loco_anymal
  anymal_model
  robot_utils
  signal_logger
  robot_control
  loco
  motion_generation_loco
  numopt_quadprog
  anymal_motion_control
  message_logger
  motion_generation
  swing_trajectory_generation
  zmp_optimizer
  tinyxml_tools
  terrain_sensing
  state_checker
  inverse_kinematics_control
  loco_perf
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########
add_definitions(-std=c++11 -Wall)

# Build the core library.
add_library(${PROJECT_NAME}
  src/DynamicGaitsController.cpp
  src/modules/LimbCoordinatorOpt.cpp
  src/modules/EventDetectorOpt.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

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
