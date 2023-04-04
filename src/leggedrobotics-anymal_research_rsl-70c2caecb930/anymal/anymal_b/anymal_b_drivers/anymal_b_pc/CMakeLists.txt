cmake_minimum_required(VERSION 3.5.1)
project(anymal_b_pc)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY install
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
