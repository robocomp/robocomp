cmake_minimum_required(VERSION 3.10)
project(dsr
        VERSION 2021.3.1
        DESCRIPTION "Robocomp DSR library"
        LANGUAGES CXX)

include(GNUInstallDirs)

add_subdirectory(api)
add_subdirectory(core)
add_subdirectory(gui)
#add_subdirectory(python-wrapper)

add_library(dsr SHARED)
add_library(Robocomp::dsr ALIAS dsr)
target_link_libraries(dsr
        PUBLIC
        Robocomp::dsr_api
        Robocomp::dsr_gui
        Robocomp::dsr_core
        )

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS dsr EXPORT dsrConfig
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT dsrConfig DESTINATION share/robocomp/cmake)

# This makes the project importable from the build directory
export(TARGETS dsr FILE dsrConfig.cmake)