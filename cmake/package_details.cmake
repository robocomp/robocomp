message(STATUS "CONFIGURING PACKAGE")

if(NOT SRC_DIR OR NOT MY_VERSION)
    set(SRC_DIR ${CMAKE_SOURCE_DIR})
    set(MY_VERSION ${RoboComp_VERSION})
endif(NOT SRC_DIR OR NOT MY_VERSION)

message(STATUS "SRC_DIR is = ${SRC_DIR}")
message(STATUS "Version  = " ${MY_VERSION})

set(CPACK_GENERATOR "DEB" CACHE STRING "generator" )
set(CPACK_PACKAGE_CONTACT "Luis J. Manso <lmanso@unex.es>" CACHE STRING "email")
set(CPACK_PACKAGE_NAME "robocomp" CACHE STRING "name")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Robotics Framework" CACHE STRING "descr")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${SRC_DIR}/debian/README.Debian" CACHE STRING "package desc file")
set(CPACK_PACKAGE_VENDOR "https://github.com/robocomp/robocomp" CACHE STRING "web")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel" CACHE STRING "sect")
SET(CPACK_RESOURCE_FILE_LICENSE "${SRC_DIR}/debian/copyright" CACHE STRING "copyright file")
SET(CPACK_PACKAGE_VERSION ${MY_VERSION} CACHE STRING "version")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional" CACHE STRING "priority")
set(CPACK_DEBIAN_DISTRIBUTION_NAME ubuntu CACHE STRING "dist-name")
set(CPACK_DEBIAN_DISTRIBUTION_RELEASES trusty vivid CACHE STRING "dists")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${SRC_DIR}/debian/postinst;${SRC_DIR}/debian/preinst;${SRC_DIR}/debian/postrm"  CACHE STRING "control extra" )
set(DEB_SRC_DIR ${SRC_DIR} CACHE STRING "src dir" )
set(CPACK_DEBIAN_CHANGELOG "  * Latest development version." CACHE STRING "name")
set(CPACK_SOURCE_IGNORE_FILES .git)
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${MY_VERSION}-${CPACK_DEBIAN_DISTRIBUTION_NAME}")
set(PPA_PGP_KEY "Luis J. Manso" CACHE STRING "1CF615C5")
set(DEB_SOURCE_CHANGES "CHANGED" CACHE STRING "source chaged since last upload")

#get the architecture of the machine
execute_process(
    COMMAND dpkg --print-architecture
    OUTPUT_VARIABLE DEB_ARCHITECTURE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "architecture is: ${DEB_ARCHITECTURE}")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${DEB_ARCHITECTURE} CACHE STRING "cpack arch")

#find and set dependencies (build and package)
set(DEBSRC_BUILD_DEPENDS debhelper cmake libgsl0-dev libopenscenegraph-dev cmake-qt-gui freeglut3-dev libboost-system-dev libboost-thread-dev openjdk-7-jre libxt-dev pyqt4-dev-tools qt4-designer qt4-dev-tools zeroc-ice35 yakuake python-pip python-pyparsing python-numpy python-pyside pyside-tools libqt4-dev qt4-qmake libqt4-opengl-dev CACHE STRING "build-dep")
set(DEBSRC_PACKAGE_DEPENDS
    libgsl23
    libopenscenegraph-dev
    libqt5opengl5-dev
    libzeroc-ice3.7
    libzeroc-icestorm3.7
    python3-prompt-toolkit
    python3-termcolor
    sudo
    zeroc-icebox
    CACHE STRING "name"
        )

#As CPack wants dependencies as a single comma separated string
set(CPACK_DEBIAN_PACKAGE_DEPENDS)
foreach(DEP ${DEBSRC_PACKAGE_DEPENDS})
    if(CPACK_DEBIAN_PACKAGE_DEPENDS)
        set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, ${DEP}")
    else()
        set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEP})
    endif()
endforeach(DEP ${DEBSRC_PACKAGE_DEPENDS})  

set(CPACK_DEBIAN_BUILD_DEPENDS)
foreach(DEP ${DEBSRC_BUILD_DEPENDS})
    if(CPACK_DEBIAN_BUILD_DEPENDS)
        set(CPACK_DEBIAN_BUILD_DEPENDS "${CPACK_DEBIAN_BUILD_DEPENDS}, ${DEP}")
    else()
        set(CPACK_DEBIAN_BUILD_DEPENDS ${DEP})
    endif()
endforeach(DEP ${DEBSRC_BUILD_DEPENDS})

#generate changelog

message(STATUS "PACKAGE CONFIGURATION DONE")
