# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(dynamixel_ros)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_updater
    dynamixel_ros_msgs
    libdynamixel
    param_io
    rosbash
    roscpp
    rospy
    sensor_msgs
    signal_handler
    std_msgs
    std_srvs
)

# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    diagnostic_updater
    dynamixel_ros_msgs
    libdynamixel
    param_io
    rosbash
    roscpp
    rospy
    sensor_msgs
    signal_handler
    std_msgs
    std_srvs
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(
  ${PROJECT_NAME}
  src/DynamixelNode.cpp
)

# Declare a cpp executable
add_executable(
  ${PROJECT_NAME}_node
  src/dynamixel_node.cpp
)

# Specify dependencies on automatically generated service files
add_dependencies(
  ${PROJECT_NAME}
  dynamixel_ros_msgs_generate_messages_cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY conf launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
