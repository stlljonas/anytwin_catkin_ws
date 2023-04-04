# Set CMAKE_INSTALL_* if not defined
set(CMAKECONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${LRS_TARGET}")

add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

include(CMakePackageConfigHelpers)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/realsense2ConfigVersion.cmake"
    VERSION ${REALSENSE_VERSION_STRING} COMPATIBILITY AnyNewerVersion)

configure_package_config_file(CMake/realsense2Config.cmake.in realsense2Config.cmake
    INSTALL_DESTINATION ${CMAKECONFIG_INSTALL_DIR}
    INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/bin
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR
)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
configure_file(config/librealsense.pc.in config/realsense2.pc @ONLY)

install(TARGETS ${LRS_TARGET}
    EXPORT realsense2Targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/any_librealsense2/
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/any_librealsense2/
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/any_librealsense2/
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/any_librealsense2"
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/any_librealsense2
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/any_librealsense2/
)

install(EXPORT realsense2Targets
        FILE realsense2Targets.cmake
        NAMESPACE ${LRS_TARGET}::
        DESTINATION ${CMAKECONFIG_INSTALL_DIR}/any_librealsense2/
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/realsense2Config.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR}/any_librealsense2/
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/realsense2ConfigVersion.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR}/any_librealsense2/
)

# Set library pkgconfig file for facilitating 3rd party integration
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config/realsense2.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig/any_librealsense2/"
)

install(CODE "execute_process(COMMAND ldconfig)")
