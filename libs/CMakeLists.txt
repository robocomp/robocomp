cmake_minimum_required( VERSION 3.10 )
project(robocomp_libs
        VERSION 2021.3.1
        DESCRIPTION "Robocomp libs"
        LANGUAGES CXX )

include(GNUInstallDirs)

add_subdirectory(qmat)

add_subdirectory(osgviewer)

add_subdirectory(innermodel)

# TEST app for the libs
add_executable(testlibs EXCLUDE_FROM_ALL)
target_sources(testlibs
        PRIVATE
        test_libs.cpp
        )

target_link_libraries(testlibs
        PRIVATE
        Robocomp::innermodel
        Robocomp::osgviewer
        )

IF(DSR)
    add_subdirectory(dsr)
    find_package(Qt5 COMPONENTS Widgets REQUIRED)
    add_executable(testdsr)
    target_sources(testdsr
            PRIVATE
            test_dsr.cpp
            )

    target_link_libraries(testdsr
            PRIVATE
            Robocomp::dsr_api
            Robocomp::dsr_gui
            Qt5::Widgets
            )
ENDIF(DSR)

#add_subdirectory(innermodel-python3)





