cmake_minimum_required( VERSION 3.10 )
project(rcosgviewer
		VERSION 2021.2.1
		DESCRIPTION "RoboComp OsgViewer library"
		LANGUAGES CXX )

find_package(Qt5 COMPONENTS Gui Xml OpenGL REQUIRED)
# OPENSG
find_package(OpenSceneGraph REQUIRED COMPONENTS osg osgDB osgGA osgUtil osgViewer)


add_library(rcosgviewer SHARED)

target_sources(rcosgviewer
		PRIVATE
		adapterwidget.cpp
		findnamednode.cpp
		getworldcoorofnode.cpp
		osgview.cpp
		viewerqt.cpp
		)

target_link_libraries(rcosgviewer
		PUBLIC ${OPENSCENEGRAPH_LIBRARIES}
		${OSGDB_LIBRARY} ${OSGVIEWER_LIBRARY}
		PUBLIC Qt5::Gui Qt5::Xml Qt5::OpenGL
		PUBLIC qmat
		)


# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(rcosgviewer
		PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
		)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS rcosgviewer
		EXPORT rcosgviewerConfig
		ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
		PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
		LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT rcosgviewerConfig DESTINATION share/robocomp/cmake)

# This makes the project importable from the build directory
export(TARGETS rcosgviewer NAMESPACE Robocomp FILE rcosgviewerConfig.cmake)