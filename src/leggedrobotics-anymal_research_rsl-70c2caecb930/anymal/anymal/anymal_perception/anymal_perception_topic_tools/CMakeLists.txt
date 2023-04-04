cmake_minimum_required(VERSION 3.5.1)
project(anymal_perception_topic_tools)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    launch
    scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES
    README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      roslaunch
  )
  roslaunch_add_file_check(launch)
endif()
