# Project configuration
cmake_minimum_required (VERSION 2.8)
project(free_gait_python)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    free_gait_msgs
)

catkin_package(
  #INCLUDE_DIRS
  #LIBRARIES
  #CATKIN_DEPENDS
  #DEPENDS
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

#############
##   Test  ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(cmake_python_coverage REQUIRED)
  add_pytest_with_coverage(test/test_action.py free_gait)
  add_rostest_pytest_with_coverage(test/action.test)
endif()
