cmake_minimum_required(VERSION 3.5.1)
project(any_description)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()


#############
## Install ##
#############
install(DIRECTORY urdf config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
