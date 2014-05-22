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
    include/${LIB_NAME}/QMatAll
   )

set(OUT_FILE_NAME ${PROJECT_NAME_LOWER}_${LIB_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include )

INCLUDE ( $ENV{ROBOCOMP}/cmake/modules/ipp.cmake )

INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})

RoboComp_ADD_LIBRARY(${OUT_FILE_NAME}  SHARED ${LIB_NAME}  ${srcs} ${incs} )
RoboComp_ADD_INCLUDES(${LIB_NAME} ${LIB_NAME} ${incs})


target_link_libraries(${OUT_FILE_NAME} ${QT_LIBRARIES} -lgsl -lgslcblas -lm ${IPP_LIBS_PATH} ${IPP_LIBS})

