cmake_minimum_required( VERSION 2.8 )

SET ( SOURCES
	adapterwidget.cpp
	findnamednode.cpp
	getworldcoorofnode.cpp
	osgview.cpp
	viewerqt.cpp
)

SET ( HEADERS
	adapterwidget.h
	findnamednode.h
	getworldcoorofnode.h
	osgview.h
	viewerqt.h
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )

# Qt
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )

# OSG
FIND_PACKAGE( osg REQUIRED )

# Specify construction and link process
#SET (LIBRARY_OUTPUT_PATH .)
ADD_LIBRARY( robocomp_osgviewer SHARED ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${OSG_LIBRARY} )

INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/librobocomp_osgviewer.so DESTINATION /opt/robocomp/lib/ )
INSTALL(FILES ${HEADERS} DESTINATION /opt/robocomp/include/osgviewer )
