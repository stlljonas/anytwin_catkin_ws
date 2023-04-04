cmake_minimum_required(VERSION 3.5.1)
project(anymal_logging)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
    launch
    plot_juggler
    rqt_multiplots
    signal_logger
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS
  bin/create_rosbag_info_files.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
