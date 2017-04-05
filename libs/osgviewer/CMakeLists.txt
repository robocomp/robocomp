set(LIB_NAME osgviewer)
set(LIB_DESC "osgviewer library")
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
	include/${LIB_NAME}/osgview.h
	include/${LIB_NAME}/viewerqt.h
   )

RoboComp_LIB_DEPEND(build ${LIB_NAME} DEPS ${LIB_DEPS} OPT_DEPS)
set(OUT_FILE_NAME ${PROJECT_NAME_LOWER}_${LIB_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTOPENGL TRUE )

QT4_WRAP_CPP( MOC_SOURCES ${incs} )

INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})



# Python bindings
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
if (PYTHON_BINDINGS_SUPPORT)
	# Add bindings source code file
	SET(srcs ${srcs} osgviewer_python.cpp)
	find_package(Boost COMPONENTS system thread python REQUIRED)
	find_package(PythonLibs REQUIRED)
	MESSAGE(STATUS "im PYTHON BINDINGS: DIRS ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}")
	MESSAGE(STATUS "im PYTHON BINDINGS: LIBS ${PYTHON_LIBRARIES} ${Boost_LIBRARY_DIRS_DIRS}")
	link_directories(${PYTHON_LIBRARIES} ${Boost_LIBRARY_DIRS_DIRS})
endif (PYTHON_BINDINGS_SUPPORT)

RoboComp_ADD_LIBRARY(${OUT_FILE_NAME} SHARED ${LIB_NAME} ${srcs} ${MOC_SOURCES})
RoboComp_ADD_INCLUDES(${LIB_NAME} ${LIB_NAME} ${incs})



target_link_libraries(${OUT_FILE_NAME} ${QT_LIBRARIES} -losg -losgViewer  ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} robocomp_qmat)



# Python bindings
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)
if (PYTHON_BINDINGS_SUPPORT)
	# Add bindings source code file
	SET(srcs ${srcs} innermodel_python.cpp)
	#find_package(Boost COMPONENTS system thread python REQUIRED)
	SET(Boost_LIBRARIES -lboost_system -lboost_thread -lboost_python)
	include_directories(${Boost_INCLUDE_DIRS})
	link_directories(${Boost_LIBRARY_DIRS})
	find_package(PythonLibs REQUIRED)
	MESSAGE(STATUS "osgv PYTHON BINDINGS: DIRS ${PYTHON_INCLUDE_DIRS}")
	MESSAGE(STATUS "osgv PYTHON BINDINGS: LIBS ${PYTHON_LIBRARIES}")
	include_directories(${PYTHON_INCLUDE_DIRS})
	link_directories(${PYTHON_LIBRARIES})
endif (PYTHON_BINDINGS_SUPPORT)
