cmake_minimum_required(VERSION 3.5.1)
project(hri_user_interface)

add_definitions(-std=c++14 -Wall -Werror)

find_package(
  catkin REQUIRED
  COMPONENTS
    any_node
    anydrive_msgs
    anymal_lowlevel_controller_common
    anymal_msgs
    hri_safety_sense
    joy_manager_msgs
    motion_transitioner_msgs
    notification
    parameter_handler_msgs
    parameter_handler
    roscpp
    sensor_msgs
    std_msgs
    rocoma_msgs
    param_io
    std_srvs
)

find_package(Boost REQUIRED COMPONENTS chrono thread)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    hri_user_interface
  CATKIN_DEPENDS
    any_node
    anydrive_msgs
    anymal_lowlevel_controller_common
    anymal_msgs
    hri_safety_sense
    joy_manager_msgs
    motion_transitioner_msgs
    notification
    parameter_handler_msgs
    parameter_handler
    sensor_msgs
    std_msgs
    roscpp
    rocoma_msgs
    param_io
    std_srvs
  DEPENDS
    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(
  hri_user_interface
    src/AnymalMotionControlForm.cpp
    src/Buttons.cpp
    src/Form.cpp
    src/HRIUserInterface.cpp
    src/LineWidget.cpp
    src/ParameterLineWidget.cpp
    src/ServiceForm.cpp
    src/TextLineWidget.cpp
    src/StatusLineWidget.cpp
    src/EmergencyForm.cpp
)
add_dependencies(hri_user_interface ${catkin_EXPORTED_TARGETS})

target_link_libraries(hri_user_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(hri_user_interface_node
  src/hri_user_interface_node.cpp
)

add_dependencies(hri_user_interface_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(hri_user_interface_node
  hri_user_interface
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
  add_rostest_gtest(${PROJECT_NAME}-test
    test/hri_user_interface.test
    test/test_main.cpp
    test/HRIUserInterfaceTest.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
    pthread
  )
endif()

install(TARGETS hri_user_interface hri_user_interface_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
