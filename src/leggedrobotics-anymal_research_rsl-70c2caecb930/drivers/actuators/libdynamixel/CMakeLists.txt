# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(libdynamixel)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS)

# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Set output paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Declare a cpp library
add_library(
  ${PROJECT_NAME}
  src/com/BinaryReader.cpp
  src/com/BinaryWriter.cpp
  src/com/SerialPort.cpp
  src/exceptions/Exception.cpp
  src/exceptions/IOException.cpp
  src/exceptions/SystemException.cpp
  src/sensor/Controller.cpp
  src/sensor/Packet.cpp
)

# Declare a cpp executable
add_executable(
  serial_test
  src/console_tools/serialTest.cpp
)
add_executable(
  set_baud_rate
  src/console_tools/setBaudRate.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(
  serial_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(
  set_baud_rate
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} serial_test set_baud_rate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)