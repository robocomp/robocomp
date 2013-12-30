cmake_minimum_required( VERSION 2.6 )

# IPP
INCLUDE( $ENV{ROBOCOMP}/cmake/ipp.cmake )
INCLUDE_DIRECTORIES( . .. )

ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )

SET ( HEADERS
  include/QMat/qvec.h 
  include/QMat/qessential.h 
  include/QMat/qrtmat.h 
  include/QMat/qmovingrobot.h 
  include/QMat/qfundamental.h 
  include/QMat/qmatrot.h 
  include/QMat/qcamera.h 
  include/QMat/qhomo.h 
  include/QMat/qextrinsics.h 
  include/QMat/qmat.h 
  include/QMat/quaternion.h 
  include/QMat/QMatAll
  )

# Sources set
SET ( SOURCES
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


# Specify construction and link process
# SET (LIBRARY_OUTPUT_PATH .)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include )
add_library(qmat SHARED ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} )
target_link_libraries(qmat ${LIBS})

INSTALL(FILES ${CMAKE_BINARY_DIR}/libqmat.so DESTINATION /opt/robocomp/lib/ )
INSTALL(FILES ${HEADERS} DESTINATION /opt/robocomp/include/QMat )
