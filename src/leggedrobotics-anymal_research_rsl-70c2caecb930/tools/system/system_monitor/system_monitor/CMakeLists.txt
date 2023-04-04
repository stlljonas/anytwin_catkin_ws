cmake_minimum_required(VERSION 3.5.1)
project(system_monitor)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS
    bin/chrony_monitor.py
    bin/cpu_monitor.py
    bin/disk_monitor.py
    bin/memory_monitor.py
    bin/network_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
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
