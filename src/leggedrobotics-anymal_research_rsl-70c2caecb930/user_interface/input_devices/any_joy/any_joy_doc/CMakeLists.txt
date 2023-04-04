cmake_minimum_required(VERSION 3.5.1)
project(any_joy_doc)

#--------------------------#
# Documentation

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    joy_interface
    joy_manager
    joy_manager_msgs
    rqt_joypad
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      joy_interface
      joy_manager
      joy_manager_msgs
      rqt_joypad
    DEPENDS doxygen
  )


  set(DOXYGEN_SOURCE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../joy_interface/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../joy_manager/\"")
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(DOXYGEN_IMAGE_DIRS "\"${DOXYGEN_SOURCE_DIRS}\"" )
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen
  execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)
  #ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${SOURCE_JOY_INTERFACE_DIR} CACHE)
  unset(${SOURCE_JOY_MANAGER_DIR} CACHE)
#  unset(${SOURCE_RQT_JOYPAD_DIR} CACHE)
ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
