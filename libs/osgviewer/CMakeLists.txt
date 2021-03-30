cmake_minimum_required( VERSION 3.10 )
project(osgviewer
		VERSION 2021.2.1
		DESCRIPTION "RoboComp OsgViewer library"
		LANGUAGES CXX )

find_package(Qt5 COMPONENTS Gui Xml OpenGL Core REQUIRED)
# OPENSG
find_package(OpenSceneGraph REQUIRED COMPONENTS osg osgDB osgGA osgUtil osgViewer osgText)


add_library(osgviewer SHARED)

target_sources(osgviewer
		PRIVATE
		adapterwidget.cpp
		findnamednode.cpp
		getworldcoorofnode.cpp
		osgview.cpp
		viewerqt.cpp
		)

target_link_libraries(osgviewer
		PUBLIC ${OPENSCENEGRAPH_LIBRARIES}
		PUBLIC Qt5::Gui Qt5::Xml Qt5::OpenGL Qt5::Core
		PUBLIC qmat
		)


# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(osgviewer
		PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>  # <prefix>/include/mylib
		)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS osgviewer
		EXPORT osgviewerConfig
		ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
		PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
		LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT osgviewerConfig DESTINATION share/robocomp/cmake)

# This makes the project importable from the build directory
export(TARGETS osgviewer NAMESPACE Robocomp FILE osgviewerConfig.cmake)