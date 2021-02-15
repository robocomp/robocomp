cmake_minimum_required( VERSION 3.12 )
project(innermodel)
set(LIB_DESC "RoboComp InnerModel library")


find_package(Qt5 COMPONENTS Core Gui REQUIRED)

add_library(innermodel)
target_sources(innermodel
		PRIVATE innermodel.cpp
		innermodelreader.cpp
		innermodelviewer.cpp
		innermodeldraw.cpp
		innermodelexception.cpp
		innermodelnode.cpp
		innermodeltransform.cpp
		innermodeljoint.cpp
		innermodelomnirobot.cpp
		innermodeldifferentialrobot
		innermodelprismaticjoint.cpp
		innermodelplane.cpp
		innermodelcamera.cpp
		innermodelrgbd.cpp
		innermodellaser.cpp
		innermodelimu.cpp
		innermodelpointcloud.cpp
		innermodelmesh.cpp
		innermodeltouchsensor.cpp
		innermodeldisplay.cpp
		PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/innermodel/innermodel.h)
target_link_libraries(innermodel
		PUBLIC Qt5::Core Qt5::Gui
		PUBLIC qmat
		PUBLIC osgviewer)
target_include_directories(innermodel
		PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

## FCL
#SET(FCL_SUPPORT ON CACHE BOOL "Build FCL shared library" FORCE)
#MESSAGE(STATUS "FCL_SUPPORT Build with FCL support ${FCL_SUPPORT}")

if (FCL_SUPPORT)
	target_link_libraries(innermodel
			PRIVATE fcl )
	target_compile_definitions(innermodel
			PRIVATE FCL_SUPPORT=1)
	MESSAGE( STATUS "COMPILING WITH FCL SUPPORT=1" )
else (FCL_SUPPORT)
	MESSAGE( STATUS "COMPILING WITHOUT FCL SUPPORT=0" )
endif (FCL_SUPPORT)

add_library(Robocomp::innermodel ALIAS innermodel)