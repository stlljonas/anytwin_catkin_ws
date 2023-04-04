cmake_minimum_required(VERSION 3.5.1)
project(kernel_monitor)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  PROGRAMS
    bin/kernel_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
