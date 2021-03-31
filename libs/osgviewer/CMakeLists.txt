cmake_minimum_required( VERSION 3.10 )
project(osgviewer
		VERSION 2021.3.1
		DESCRIPTION "RoboComp OsgViewer library"
		LANGUAGES CXX
		)

# QT
find_package(Qt5 COMPONENTS Gui Xml OpenGL Core REQUIRED)

# OPENSG
find_package(OpenSceneGraph REQUIRED COMPONENTS osg osgDB osgGA osgUtil osgViewer osgText)

add_library(osgviewer SHARED )
add_library(Robocomp::osgviewer ALIAS osgviewer )

set_property(TARGET osgviewer PROPERTY AUTOMOC ON)
set_property(TARGET osgviewer PROPERTY AUTOUIC ON)

# Needed to get automoc process the *.h files
file(GLOB headers_to_moc
		${CMAKE_CURRENT_SOURCE_DIR}/include/osgviewer/*.h
		)

target_sources(osgviewer
		PRIVATE
		adapterwidget.cpp
		findnamednode.cpp
		getworldcoorofnode.cpp
		osgview.cpp
		viewerqt.cpp
		${headers_to_moc})

target_link_libraries(osgviewer
		PRIVATE Robocomp::qmat
		PRIVATE Qt5::Xml Qt5::Core
		PUBLIC  Qt5::Gui Qt5::Widgets Qt5::OpenGL
		PUBLIC ${OPENSCENEGRAPH_LIBRARIES}
		)

target_include_directories(osgviewer
		PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
		$<INSTALL_INTERFACE:include/>  # <prefix>/include/mylib
		PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(osgviewer
		PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
		$<INSTALL_INTERFACE:include/>
		)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS osgviewer EXPORT osgviewerConfig
		ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
		)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT osgviewerConfig DESTINATION share/robocomp/cmake)

# This makes the project importable from the build directory
export(TARGETS osgviewer FILE osgviewerConfig.cmake)