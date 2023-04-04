cmake_minimum_required(VERSION 3.5.1)
project(anymal_b_perception_replay)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      roslaunch
  )
  roslaunch_add_file_check(launch)
endif()
