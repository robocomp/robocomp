set(LIB_NAME innermodel)
set(LIB_DESC "RoboComp InnerModel library")
set(LIB_DEPS qmat osgviewer)

set(srcs
	innermodel.cpp
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
)

set(incs
    ${CMAKE_CURRENT_BINARY_DIR}/include/${LIB_NAME}/innermodelconfig.h
    include/${LIB_NAME}/innermodel.h
    include/${LIB_NAME}/innermodelreader.h
    include/${LIB_NAME}/innermodelviewer.h
    include/${LIB_NAME}/innermodeldraw.h
    include/${LIB_NAME}/innermodelcamera.h
    include/${LIB_NAME}/innermodeldifferentialrobot.h
    include/${LIB_NAME}/innermodelexception.h
    include/${LIB_NAME}/innermodelimu.h
    include/${LIB_NAME}/innermodeljoint.h
    include/${LIB_NAME}/innermodelmesh.h
    include/${LIB_NAME}/innermodelnode.h
    include/${LIB_NAME}/innermodelomnirobot.h
    include/${LIB_NAME}/innermodelplane.h
    include/${LIB_NAME}/innermodelpointcloud.h
    include/${LIB_NAME}/innermodelprismaticjoint.h
    include/${LIB_NAME}/innermodellaser.h
    include/${LIB_NAME}/innermodelrgbd.h
    include/${LIB_NAME}/innermodeltouchsensor.h
    include/${LIB_NAME}/innermodeltransform.h
    include/${LIB_NAME}/innermodelmesh.h
    include/${LIB_NAME}/innermodeldisplay.h
)

# FCL
option (FCL_SUPPORT "Build with FCL support" OFF)
if (FCL_SUPPORT)
	SET (FCL_LIBS -lfcl )
	ADD_DEFINITIONS(-DFCL_SUPPORT=1)
	MESSAGE( STATUS "COMPILING WITH FCL SUPPORT  =  1" )
	SET (FCL_SUPPORT_VALUE "1")
else (FCL_SUPPORT)
	MESSAGE( STATUS "COMPILING WITHOUT FCL SUPPORT  = 0" )
	SET (FCL_SUPPORT_VALUE "0")
endif (FCL_SUPPORT)

include_directories( ${CMAKE_CURRENT_BINARY_DIR}/include include/innermodel )
configure_file ("include/${LIB_NAME}/innermodelconfig.in.h"  "include/${LIB_NAME}/innermodelconfig.h" )

# Python bindings
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
if (PYTHON_BINDINGS_SUPPORT)
	# Add bindings source code file
	SET(srcs ${srcs} innermodel_python.cpp)
	find_package(Boost COMPONENTS system thread python REQUIRED)
	find_package(PythonLibs 2.7 REQUIRED)
	MESSAGE(STATUS "im PYTHON BINDINGS: DIRS ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}")
	MESSAGE(STATUS "im PYTHON BINDINGS: LIBS ${PYTHON_LIBRARIES} ${Boost_LIBRARY_DIRS_DIRS}")
	link_directories(${PYTHON_LIBRARIES} ${Boost_LIBRARY_DIRS_DIRS})
endif (PYTHON_BINDINGS_SUPPORT)

RoboComp_LIB_DEPEND(build ${LIB_NAME} DEPS ${LIB_DEPS} OPT_DEPS)
set(OUT_FILE_NAME ${PROJECT_NAME_LOWER}_${LIB_NAME})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include  ${CMAKE_CURRENT_BINARY_DIR}/libs ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})

RoboComp_ADD_LIBRARY(${OUT_FILE_NAME} SHARED ${LIB_NAME} ${srcs} ${incs} )
RoboComp_ADD_INCLUDES(${LIB_NAME} ${LIB_NAME} ${incs})

target_link_libraries( ${OUT_FILE_NAME} ${QT_LIBRARIES} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} -losg -losgViewer robocomp_qmat ${FCL_LIBS} )
