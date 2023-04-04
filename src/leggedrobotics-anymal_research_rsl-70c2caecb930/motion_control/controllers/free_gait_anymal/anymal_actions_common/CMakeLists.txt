cmake_minimum_required(VERSION 3.5.1)
project(anymal_actions_common)

find_package(catkin REQUIRED
)

catkin_package(
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

catkin_python_setup()

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
