cmake_minimum_required( VERSION 3.12 )
project(osgviewer)

find_package(Qt5 COMPONENTS Gui REQUIRED)

add_library(osgviewer SHARED)
target_sources(osgviewer
		PRIVATE adapterwidget.cpp
		findnamednode.cpp
		getworldcoorofnode.cpp
		osgview.cpp
		viewerqt.cpp
		PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/osgviewer/osgview.h
		PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/osgviewer/viewerqt.h
		PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/osgviewer/adapterwidget.h
		)
target_link_libraries(osgviewer
		PUBLIC Qt5::Gui
		PUBLIC qmat )
target_include_directories(osgviewer
		PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")


add_library( Robocomp::osgviewer ALIAS osgviewer )