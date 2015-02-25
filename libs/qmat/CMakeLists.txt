set(LIB_NAME qmat)
set(LIB_DESC "RoboComp qmat library")


set(srcs
    qfundamental.cpp
    qessential.cpp
    qcamera.cpp
    qextrinsics.cpp
    qmat.cpp
    qmovingrobot.cpp
    qvec.cpp
    qmatrot.cpp
    qrtmat.cpp
    qhomo.cpp
    qline2d.cpp
   )

set(incs
    include/${LIB_NAME}/qvec.h 
    include/${LIB_NAME}/qessential.h 
    include/${LIB_NAME}/qrtmat.h 
    include/${LIB_NAME}/qmovingrobot.h 
    include/${LIB_NAME}/qfundamental.h 
    include/${LIB_NAME}/qmatrot.h 
    include/${LIB_NAME}/qcamera.h 
    include/${LIB_NAME}/qhomo.h 
    include/${LIB_NAME}/qextrinsics.h 
    include/${LIB_NAME}/qmat.h 
    include/${LIB_NAME}/quaternion.h 
    include/${LIB_NAME}/qline2d.h
    include/${LIB_NAME}/QMatAll
   )


   
   
# Python bindings
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)
if (PYTHON_BINDINGS_SUPPORT)
	# Add bindings source code file
	SET(srcs ${srcs} qmat_python.cpp)
	#find_package(Boost COMPONENTS system thread python REQUIRED)
	SET(Boost_LIBRARIES -lboost_system -lboost_thread -lboost_python )
	include_directories(${Boost_INCLUDE_DIRS})
	link_directories(${Boost_LIBRARY_DIRS})
	find_package(PythonLibs REQUIRED)
	MESSAGE(STATUS "PYTHON BINDINGS: DIRS ${PYTHON_INCLUDE_DIRS}")
	MESSAGE(STATUS "PYTHON BINDINGS: LIBS ${PYTHON_LIBRARIES}")
	include_directories(${PYTHON_INCLUDE_DIRS})
	link_directories(${PYTHON_LIBRARIES})
endif (PYTHON_BINDINGS_SUPPORT)


set(OUT_FILE_NAME ${PROJECT_NAME_LOWER}_${LIB_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include )

#INCLUDE ( $ENV{ROBOCOMP}/cmake/modules/ipp.cmake )

INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})

RoboComp_ADD_LIBRARY(${OUT_FILE_NAME}  SHARED ${LIB_NAME}  ${srcs} ${incs} )
RoboComp_ADD_INCLUDES(${LIB_NAME} ${LIB_NAME} ${incs})


target_link_libraries(${OUT_FILE_NAME} ${QT_LIBRARIES} -lgsl -lgslcblas -lm ${IPP_LIBS_PATH} ${IPP_LIBS} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})

