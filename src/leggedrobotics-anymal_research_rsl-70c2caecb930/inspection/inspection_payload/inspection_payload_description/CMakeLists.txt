cmake_minimum_required(VERSION 3.3)
project(inspection_payload_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch mesh urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
