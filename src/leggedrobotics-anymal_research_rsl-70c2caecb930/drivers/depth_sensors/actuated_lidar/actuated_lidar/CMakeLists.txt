# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(actuated_lidar)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    actuated_lidar_msgs
    angles
    dynamixel_ros_msgs
    dynamixel_ros
    laser_assembler
    laser_filters
    laser_geometry
    param_io
    pcl_ros
    robot_state_publisher
    rosbash
    roscpp
    rospy
    scan_to_pointcloud
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    urg_node
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
    actuated_lidar_msgs
    angles
    dynamixel_ros_msgs
    dynamixel_ros
    laser_assembler
    laser_filters
    laser_geometry
    param_io
    pcl_ros
    robot_state_publisher
    rosbash
    roscpp
    rospy
    scan_to_pointcloud
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    urg_node
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(
  ${PROJECT_NAME}
  src/ActuatedLidar.cpp
  src/ActuatedLidarMaster.cpp
)

# Declare a cpp executable
add_executable(
  ${PROJECT_NAME}_node
  src/actuated_lidar_node.cpp
)

# Declare a cpp executable (master node)
add_executable(
  ${PROJECT_NAME}_master_node
  src/actuated_lidar_master_node.cpp
)

# Specify dependencies on automatically generated service files
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
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
target_link_libraries(
  ${PROJECT_NAME}_master_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_master_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

