cmake_minimum_required(VERSION 3.5.1)
project(anymal_piksi_rtk_gps)

find_package(catkin REQUIRED
  COMPONENTS
    piksi_multi_rtk_ros
)

catkin_package(
  CATKIN_DEPENDS
    piksi_multi_rtk_ros
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
