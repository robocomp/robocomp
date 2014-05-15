set(LIB_NAME innermodel)
set(LIB_DESC "RoboComp InnerModel library")
set(LIB_DEPS qmat osgviewer)

set(srcs
	innermodel.cpp
	innermodelreader.cpp
	innermodelviewer.cpp
)

set(incs
    include/${LIB_NAME}/innermodel.h
    include/${LIB_NAME}/innermodelreader.h
    include/${LIB_NAME}/innermodelviewer.h
)
    
# FCL
option (FCL_SUPPORT "Build with FCL support" OFF)
if (FCL_SUPPORT)
	SET (FCL_LIBS -lfcl -lccd )
	ADD_DEFINITIONS(-DFCL_SUPPORT=1)
	MESSAGE( STATUS "COMPILING WITH FCL SUPPORT" )
	SET (FCL_SUPPORT_VALUE "1")
else (FCL_SUPPORT)
	MESSAGE( STATUS "COMPILING WITHOUT FCL SUPPORT" )
	SET (FCL_SUPPORT_VALUE "0")
endif (FCL_SUPPORT)

configure_file ("include/${LIB_NAME}/innermodel.in.h" "include/${LIB_NAME}/innermodel.h")


RoboComp_LIB_DEPEND(build ${LIB_NAME} DEPS ${LIB_DEPS} OPT_DEPS)
set(OUT_FILE_NAME ${PROJECT_NAME_LOWER}_${LIB_NAME})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )

INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})

RoboComp_ADD_LIBRARY(${OUT_FILE_NAME} SHARED ${LIB_NAME} ${srcs} ${incs} )
RoboComp_ADD_INCLUDES(${LIB_NAME} ${LIB_NAME} ${incs})

target_link_libraries( ${OUT_FILE_NAME} ${QT_LIBRARIES} -losg -losgViewer robocomp_qmat ${FCL_LIBS})

