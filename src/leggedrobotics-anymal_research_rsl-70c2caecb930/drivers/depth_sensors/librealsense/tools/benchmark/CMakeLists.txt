# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseToolsBenchmark)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-benchmark rs-benchmark.cpp ../../third-party/glad/glad.c)
    target_link_libraries(rs-benchmark ${DEPENDENCIES} realsense2-gl)
    include_directories(rs-benchmark ../../third-party/tclap/include ../../third-party/glad)
    set_target_properties (rs-benchmark PROPERTIES
        FOLDER Tools
    )

    install(
        TARGETS

        rs-benchmark

        RUNTIME DESTINATION
        ${CMAKE_INSTALL_BINDIR}/any_librealsense2/
    )
endif()