cmake_minimum_required(VERSION 3.5.1)
project(gazebo2rviz)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(
  PROGRAMS
    scripts/gazebo2tf_node.py
    scripts/gazebo2marker_node.py
    scripts/sdf2marker_node.py
    scripts/sdf2tfstatic_node.py
    scripts/sdf2extract_tfstatic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
