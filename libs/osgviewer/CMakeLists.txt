set(LIB_NAME osgviewer)
set(LIB_DESC "osviewer library")
set(LIB_DEPS qmat)

set(srcs
	adapterwidget.cpp
	findnamednode.cpp
	getworldcoorofnode.cpp
	osgview.cpp
	viewerqt.cpp
   )

set(incs
	include/${LIB_NAME}/adapterwidget.h
	include/${LIB_NAME}/findnamednode.h
	include/${LIB_NAME}/getworldcoorofnode.h
	include/${LIB_NAME}/osgview.h
	include/${LIB_NAME}/viewerqt.h
   )
    
RoboComp_LIB_DEPEND(build ${LIB_NAME} DEPS ${LIB_DEPS} OPT_DEPS)
set(OUT_FILE_NAME ${PROJECT_NAME_LOWER}_${LIB_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTOPENGL TRUE )

INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})

RoboComp_ADD_LIBRARY(${OUT_FILE_NAME} SHARED ${LIB_NAME} ${srcs} ${incs} )
RoboComp_ADD_INCLUDES(${LIB_NAME} ${LIB_NAME} ${incs})


target_link_libraries(${OUT_FILE_NAME} ${QT_LIBRARIES} )