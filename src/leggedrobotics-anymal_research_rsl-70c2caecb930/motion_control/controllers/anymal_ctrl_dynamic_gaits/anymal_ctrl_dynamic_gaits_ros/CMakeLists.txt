# Project configuration
cmake_minimum_required (VERSION 2.8)
project(anymal_ctrl_dynamic_gaits_ros)

# List of dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  loco_ros_anymal
  anymal_ctrl_dynamic_gaits
  robot_utils
  tf
  anymal_msgs
  geometry_msgs
  visualization_msgs
  any_msgs
  robot_control
  message_logger
  motion_generation_msgs
  motion_generation_ros
  elevation_map_processing
  terrain_sensing
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_definitions(-std=c++11 -Wall)

# Build the core library.
add_library(${PROJECT_NAME} src/DynamicGaitsControllerRos.cpp)
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
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
