set(LIB_NAME innermodel)
set(LIB_DESC "RoboComp InnerModel library")
set(LIB_DEPS qmat osgviewer)

set(srcs
	innermodel.cpp
	innermodelreader.cpp
	innermodelviewer.cpp
	innermodeldraw.cpp
)

set(incs
    ${CMAKE_CURRENT_BINARY_DIR}/include/${LIB_NAME}/innermodel.h
    include/${LIB_NAME}/innermodelreader.h
    include/${LIB_NAME}/innermodelviewer.h
    include/${LIB_NAME}/innermodeldraw.h
)
    
# FCL
option (FCL_SUPPORT "Build with FCL support" OFF)
if (FCL_SUPPORT)
	INCLUDE_DIRECTORIES(/usr/local/include)
	SET (FCL_LIBS -lfcl -lccd )
	ADD_DEFINITIONS(-DFCL_SUPPORT=1)
	MESSAGE( STATUS "COMPILING WITH FCL SUPPORT  =  1" )
	SET (FCL_SUPPORT_VALUE "1")
else (FCL_SUPPORT)
	MESSAGE( STATUS "COMPILING WITHOUT FCL SUPPORT  = 0" )
	SET (FCL_SUPPORT_VALUE "0")
endif (FCL_SUPPORT)


include_directories( ${CMAKE_CURRENT_BINARY_DIR}/include include/innermodel )

configure_file ("include/${LIB_NAME}/innermodel.in.h" "include/${LIB_NAME}/innermodel.h")


# Python bindings
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
if (PYTHON_BINDINGS_SUPPORT)
	# Add bindings source code file
	SET(srcs ${srcs} innermodel_python.cpp)
	find_package(Boost COMPONENTS system thread python REQUIRED)
	find_package(PythonLibs REQUIRED)
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

target_link_libraries( ${OUT_FILE_NAME} ${QT_LIBRARIES} -losg -losgViewer robocomp_qmat ${FCL_LIBS} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})

