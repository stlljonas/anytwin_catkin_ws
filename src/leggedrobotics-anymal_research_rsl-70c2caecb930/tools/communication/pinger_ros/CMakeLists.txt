cmake_minimum_required(VERSION 3.5.1)
project(pinger_ros)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    any_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    any_msgs
)

install(PROGRAMS bin/${PROJECT_NAME}/${PROJECT_NAME}.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
