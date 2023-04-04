cmake_minimum_required(VERSION 3.5.1)
project(anymal_state_estimator_doc)

#--------------------------#
# Documentation

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Doxygen)

set(PACKAGE_DEPENDENCIES
    any_state_estimator
    anymal_state_estimator
    anymal_state_estimator_lwf
    anymal_state_estimator_tsif
)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    ${PACKAGE_DEPENDENCIES}
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      ${PACKAGE_DEPENDENCIES}
    DEPENDS
  )

  # Set some environment variables
  set(DOXYGEN_SOURCE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../../any_state_estimator/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../anymal_state_estimator/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../anymal_state_estimator_lwf/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../anymal_state_estimator_tsif/\"")
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen
  execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${DOXYGEN_SOURCE_DIRS} CACHE)

ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
