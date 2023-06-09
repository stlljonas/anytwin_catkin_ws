# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.1.3)

project(uvc_fw)

file(READ "firmware-version.h" ver)

message(STATUS "Fetching recommended firmwares:")

string(REGEX MATCH "D4XX_RC_FIRMWARE_VERSION \"([0-9]+.[0-9]+.[0-9]+.[0-9]+)\"" _ ${ver})
set(D4XX_RC_VERSION ${CMAKE_MATCH_1})
message(STATUS "D4XX_RC_VERSION: ${D4XX_RC_VERSION}")
set(D4XX_RC_SHA1 b81d8b8dcdbbf89d0b1451a3da002b255851eddc)
set(D4XX_RC_URL "http://realsense-hw-public.s3-eu-west-1.amazonaws.com/Releases/RS4xx/FW")

string(REGEX MATCH "D4XX_RECOMMENDED_FIRMWARE_VERSION \"([0-9]+.[0-9]+.[0-9]+.[0-9]+)\"" _ ${ver})
set(D4XX_FW_VERSION ${CMAKE_MATCH_1})
message(STATUS "D4XX_FW_VERSION: ${D4XX_FW_VERSION}")
set(D4XX_FW_SHA1 b81d8b8dcdbbf89d0b1451a3da002b255851eddc)
set(D4XX_FW_URL "http://realsense-hw-public.s3-eu-west-1.amazonaws.com/Releases/RS4xx/FW")


string(REGEX MATCH "SR3XX_RECOMMENDED_FIRMWARE_VERSION \"([0-9]+.[0-9]+.[0-9]+.[0-9]+)\"" _ ${ver})
set(SR3XX_FW_VERSION ${CMAKE_MATCH_1})
message(STATUS "SR3XX_FW_VERSION: ${SR3XX_FW_VERSION}")
set(SR3XX_FW_SHA1 55237dba5d7db20e7c218975375d05b4210e9460)
set(SR3XX_FW_URL "http://realsense-hw-public.s3-eu-west-1.amazonaws.com/Releases/SR300/FW")

add_library(${PROJECT_NAME} STATIC empty.c)

if (MSVC)
  # lib.exe can't handle multiple .res files, so include them in one.
  # even then, the linker won't grab a .res out of a .lib object, so it needs to be explicitly listed
  # and to find the name of the .res file (across cmake generators) we need to create our own rule. :(
  add_custom_command(TARGET ${PROJECT_NAME} BYPRODUCTS ${PROJECT_NAME}.res COMMAND ${CMAKE_RC_COMPILER} ${CMAKE_RC_FLAGS} /I . /fo "${PROJECT_NAME}.res" "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.rc")
  target_link_libraries(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.res")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER Resources)

function(target_binary url version sha1 symbol ext)
  set(binary "${CMAKE_CURRENT_BINARY_DIR}/${symbol}-${version}${ext}")
  file(DOWNLOAD "${url}/${symbol}-${version}${ext}" "${binary}"
       EXPECTED_HASH SHA1=${sha1}
       STATUS status)
  list(GET status 0 error_code)
  if (NOT ${error_code} EQUAL 0)
    message(FATAL_ERROR "Download firmwnare (${status}) - ${url}")
  else()
    message(STATUS "Download firmware ${status} for ${symbol}-${version}${ext}")
  endif()
  string(TOUPPER ${symbol} SYMBOL)
  string(REPLACE "." "," version_commas ${version})
  string(REPLACE "\\" "\\\\" binary_escaped "${binary}")
  configure_file(fw.c.in  "${CMAKE_CURRENT_BINARY_DIR}/${symbol}.c" @ONLY)
  configure_file(fw.h.in  "${CMAKE_CURRENT_BINARY_DIR}/${symbol}.h" @ONLY)
  configure_file(fw.rc.in "${CMAKE_CURRENT_BINARY_DIR}/${symbol}.rc" @ONLY)
  if (MSVC)
    enable_language(RC)
    set_source_files_properties("${PROJECT_NAME}.rc" OBJECT_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/${symbol}.rc")
    set_source_files_properties("${CMAKE_CURRENT_BINARY_DIR}/${symbol}.rc" OBJECT_DEPENDS "${binary}")
  else()
    set_source_files_properties("${CMAKE_CURRENT_BINARY_DIR}/${symbol}.c" OBJECT_DEPENDS "${binary}")
  endif()
  target_sources(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/${symbol}.c")
endfunction()

target_binary( "${D4XX_FW_URL}"  "${D4XX_FW_VERSION}"  "${D4XX_FW_SHA1}" D4XX_FW_Image .bin)
target_binary( "${D4XX_RC_URL}"  "${D4XX_RC_VERSION}"  "${D4XX_RC_SHA1}" D4XX_RC_Image .bin)
target_binary( "${SR3XX_FW_URL}"  "${SR3XX_FW_VERSION}"  "${SR3XX_FW_SHA1}" SR3XX_FW_Image .bin)

install(TARGETS ${PROJECT_NAME} EXPORT realsense2Targets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/any_librealsense2/
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/any_librealsense2/
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/any_librealsense2/
)
