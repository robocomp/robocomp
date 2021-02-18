# RoboComp global CMake
cmake_minimum_required(VERSION 3.0)
project(robocomp
        VERSION 2021.2.1
        DESCRIPTION "Robocomp framework"
        LANGUAGES CXX )

#variable_watch(Ice_LIBRARIES)
#variable_watch(CMAKE_SOURCE_DIR)


string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

#Set RoboComp directories
set(RoboComp_LIBS_DIR ${CMAKE_SOURCE_DIR}/libs)
set(RoboComp_CLASSES_DIR ${CMAKE_SOURCE_DIR}/classes)
set(RoboComp_INTERFACES_DIR ${CMAKE_SOURCE_DIR}/interfaces)
# Find universal dependencies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

# Release/Debug specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()

# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
SET(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel Maintainer."
    FORCE)

include(${CMAKE_SOURCE_DIR}/cmake/robocomp_targets.cmake)

include(${CMAKE_SOURCE_DIR}/cmake/robocomp_utils.cmake)
set(RoboComp_VERSION 2021.02.01 CACHE STRING "RoboComp version")

include(${CMAKE_SOURCE_DIR}/cmake/modules/qt.cmake)


# Add an "uninstall" target
configure_file("${CMAKE_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
               "${CMAKE_SOURCE_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
                  "${CMAKE_SOURCE_DIR}/uninstall_target.cmake")

# RoboComp global dependencies
#find_package(OpenMP REQUIRED) ## search OpenMP libs
#if(OPENMP_FOUND)
#  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#  message (STATUS "Found OpenMP")
#else(OPENMP_FOUND)
#  message (STATUS "Not found OpenMP")
#endif()


# Python binding
option (PYTHON_BINDINGS_SUPPORT "Build with Python bindings support" OFF)
if (PYTHON_BINDINGS_SUPPORT)
	MESSAGE(STATUS "PYTHON BINDINGS:        YES")
	add_definitions("-DPYTHON_BINDINGS_SUPPORT")
else (PYTHON_BINDINGS_SUPPORT)
	MESSAGE(STATUS "PYTHON BINDINGS:        NO")
endif (PYTHON_BINDINGS_SUPPORT)

#Find Doxygen and html help compiler if any
find_package(Doxygen)
if(DOXYGEN_FOUND)
  find_package(HTMLHelp)
endif(DOXYGEN_FOUND)

# Compiling options
SET(compile_dirs "")
OPTION(CORE "Compile RoboComp Core" ON)
OPTION(TOOLS "Compile RoboComp Tools" ON)
if (CORE)
	SET(compile_dirs ${compile_dirs} "libs")
endif(CORE)

add_subdirectory(libs)
add_subdirectory(tools)

#ROBOCOMP_IDSL_TO_ICE( AGMWorldModel AGMExecutive Planning AGMCommonBehavior AGMExecutiveTopic AGMExecutiveVisualizationTopic )


# Install rules necessary to allow compiling components without the robocomp's source code
INSTALL(FILES "files/robocomp.conf" DESTINATION /etc/ld.so.conf.d/)
INSTALL(DIRECTORY "interfaces" DESTINATION /opt/robocomp/ )
INSTALL(DIRECTORY "cmake" DESTINATION /opt/robocomp/ )
#INSTALL(DIRECTORY "files" DESTINATION /opt/robocomp/ )
INSTALL(DIRECTORY "classes" DESTINATION /opt/robocomp/
		PATTERN "*_autogen" EXCLUDE
		PATTERN "CMakeFiles" EXCLUDE
		PATTERN "build" EXCLUDE
		PATTERN "cmake-build" EXCLUDE
		PATTERN ".idea" EXCLUDE
		)


add_custom_target(mrproper
  COMMAND ${CMAKE_MAKE_PROGRAM} clean
   COMMAND find $ENV{ROBOCOMP} -name CMakeFiles -exec rm -rf {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name CMakeCache.txt -exec rm {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name cmake_install.cmake -exec rm {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name Makefile -exec rm {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name *.kd* -exec rm {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name *_moc* -exec rm {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name Makefile -exec rm {} "\\;"
)



# "make spackage for ppa"
#add_custom_target(spackage
#              ${CMAKE_COMMAND}
#              -D SRC_DIR:STRING=${RoboComp_SOURCE_DIR}
#              -D MY_VERSION:STRING=${RoboComp_VERSION}
#              -C ${CMAKE_SOURCE_DIR}/cmake/package_details.cmake
#              -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/source_package.cmake
#            )

#For packaging
include(${CMAKE_SOURCE_DIR}/cmake/package_details.cmake)
CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/debian/fixup_deb.sh.in" "${CMAKE_CURRENT_BINARY_DIR}/fixup_deb.sh" @ONLY IMMEDIATE)
include(CPack)
#add_custom_command(TARGET package POST_BUILD COMMAND bash fixup_deb.sh; WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} )
