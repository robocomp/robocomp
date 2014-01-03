# Qt4
ADD_DEFINITIONS( -Wall  -DQT_XML_LIB -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTSTATE TRUE )
SET( QT_USE_QTSTATEMACHINE TRUE )
INCLUDE( ${QT_USE_FILE} )

# OSG
FIND_PACKAGE(OpenThreads)
FIND_PACKAGE(osg)
FIND_PACKAGE(osgDB)
FIND_PACKAGE(osgViewer) 
INCLUDE_DIRECTORIES(${OPENTHREADS_INCLUDE_DIR} ${OSG_INCLUDE_DIR}) 
LINK_DIRECTORIES(${OSG_LIB_DIR}) 


MACRO( ROBOCOMP_INITIALIZE )
  set(RoboComp_VERSION 1.0 CACHE STRING "RoboComp version")
  #set install dirs
  set(BIN_INSTALL_DIR "/opt/robocomp-${RoboComp_VERSION}/bin")
  set(CONFIG_INSTALL_DIR "/opt/robocomp-${RoboComp_VERSION}/etc-default")
  # Set root directory
  SET( ROBOCOMP_ROOT ${ARGN} )
  MESSAGE(STATUS "RoboComp root is now set to ${ROBOCOMP_ROOT}")
  # Include path directories
  SET(ICEROOT $ENV{ICEROOT} )
  MESSAGE(STATUS "$ICEROOT=\"${ICEROOT}\"")
  IF (NOT ${ICEROOT} EQUAL "")
    SET(SLICECPP_PATH ${ICEROOT}/bin/)
	ENDIF (NOT ${ICEROOT} EQUAL "")
  INCLUDE_DIRECTORIES (
    .
    ${ARGN}/classes/
    ${ARGN}/interfaces/
    ${CMAKE_BINARY_DIR}
    ${ICEROOT}/include/
  )
  # Set interfaces directory
  SET(RoboComp_INTERFACES_DIR "${ARGN}/interfaces/")
  # Set libraries

#   SET( LIBS ${LIBS} -L/opt/robocomp/lib ${OSG_LIBRARY} ${OSGUTIL_LIBRARY} ${OSGDB_LIBRARY} ${OSGVIEWER_LIBRARY} ${OPENTHREADS_LIBRARY} robocomp_innermodel -lqmat -L${ICEROOT}/lib/ -L${ROBOCOMP_ROOT}/classes -lIce -lIceUtil -lpthread -lIceStorm -lgsl -lblas ${QT_LIBRARIES} -losgGA -losgText -losgDB -losgUtil -losg -losgViewer)
	SET( LIBS ${LIBS} -L/opt/robocomp/lib ${OSG_LIBRARY} ${OSGUTIL_LIBRARY} ${OSGDB_LIBRARY} ${OSGVIEWER_LIBRARY} ${OPENTHREADS_LIBRARY} -lqmat -L${ICEROOT}/lib/ -L${ROBOCOMP_ROOT}/classes -lIce -lIceUtil -lpthread -lIceStorm -lgsl -lblas ${QT_LIBRARIES} -losgGA -losgText -losgDB -losgUtil -losg -losgViewer innermodel )
  INCLUDE ( $ENV{ROBOCOMP}/cmake/modules/ipp.cmake )
ENDMACRO( ROBOCOMP_INITIALIZE )


MACRO( ROBOCOMP_LIBRARY )
  MESSAGE(STATUS "RoboComp libraries")
  FOREACH ( input_library ${ARGN} )
	IF( EXISTS "/opt/robocomp/lib/lib${input_library}.so")
		MESSAGE(STATUS "Adding library ${input_library} " )
		SET(ROBOCOMP_LIBS ${ROBOCOMP_LIBS} -l${input_library} )
	ELSE( EXISTS "/opt/robocomp/lib/lib${input_library}.so")
		MESSAGE(STATUS "Library ${input_library} not found in /opt/robocomp/lib" )
		IF (EXISTS "${ROBOCOMP_ROOT}/classes/lib${input_library}.so")
                	MESSAGE(STATUS "Adding library ${input_library} " )
                	SET(ROBOCOMP_LIBS ${ROBOCOMP_LIBS} -l${input_library} )
		ELSE (EXISTS "${ROBOCOMP_ROOT}/classes/lib${input_library}.so")
			MESSAGE(FATAL_ERROR "Library ${input_library} not found" )
		ENDIF (EXISTS "${ROBOCOMP_ROOT}/classes/lib${input_library}.so")
	ENDIF( EXISTS "/opt/robocomp/lib/lib${input_library}.so")
  ENDFOREACH ( input_library )
  SET( LIBS ${LIBS} -L/opt/robocomp/lib/ ${ROBOCOMP_LIBS} )
ENDMACRO( ROBOCOMP_LIBRARY )



MACRO( ROBOCOMP_WRAP_ICE )
  # External Slice source paths
  SET (EXTERNAL_SLICE "")
  SET (SLICE_PATH "$ENV{SLICE_PATH};/opt/robocomp/ThirdPartyinterfaces")
  SET (INC_ROBOCOMPSLICE_PATH "true" )
  SET (ADDITIONAL_SLICE_INCLUDE_PATH "")
  FOREACH (SPATH ${SLICE_PATH})
     MESSAGE(STATUS "ACHO ${SLICE_PATH}")
     IF( ${RoboComp_INTERFACES_DIR} STREQUAL ${SPATH})
       SET(INC_ROBOCOMPSLICE_PATH "false")
     ELSE( ${RoboComp_INTERFACES_DIR} STREQUAL ${SPATH})
			SET(ADDITIONAL_SLICE_INCLUDE_PATH ${ADDITIONAL_SLICE_INCLUDE_PATH} -I${SPATH})
     ENDIF( ${RoboComp_INTERFACES_DIR} STREQUAL ${SPATH})
  ENDFOREACH (SPATH ${SLICE_PATH})
  IF (${INC_ROBOCOMPSLICE_PATH} STREQUAL "true")
     SET (SLICE_PATH "${SLICE_PATH};${RoboComp_INTERFACES_DIR}")
  ENDIF(${INC_ROBOCOMPSLICE_PATH} STREQUAL "true")
  MESSAGE(STATUS "$SLICE_PATH=\"${SLICE_PATH}\"")
  FOREACH (SPATH ${SLICE_PATH})
    MESSAGE(STATUS "Adding ${SPATH} to the Slice directory set.")
    SET (EXTERNAL_SLICE "${EXTERNAL_SLICE} -I${SPATH} ")
  ENDFOREACH (SPATH)

  FOREACH( input_file ${ARGN} )
    SET (SLICE_FILE_FOUND "false")
	IF( input_file STREQUAL "Logger" )
		MESSAGE(STATUS "Compile with LoggerComp support." )
		ADD_DEFINITIONS(-DCOMPILE_LOGGERCOMP=1)
	ENDIF( input_file STREQUAL "Logger" )
    FOREACH (SPATH ${SLICE_PATH})
      IF (EXISTS "${SPATH}/${input_file}.ice")
        MESSAGE(STATUS "Adding rule to generate ${input_file}.cpp and ${input_file}.h from ${SPATH}/${input_file}.ice  (${SLICECPP_PATH}slice2cpp)" )
        ADD_CUSTOM_COMMAND (
          OUTPUT ${input_file}.cpp ${input_file}.h
          COMMAND ${SLICECPP_PATH}slice2cpp -I${RoboComp_INTERFACES_DIR} ${ADDITIONAL_SLICE_INCLUDE_PATH} -I. ${SPATH}/${input_file}.ice --output-dir .
          DEPENDS ${SPATH}/${input_file}.ice
          COMMENT "Generating ${input_file}.cpp and ${input_file}.h from ${input_file}.ice"
        )
        SET ( SOURCES ${SOURCES} ./${input_file}.cpp)
        SET ( SLICE_FILE_FOUND "true")
	break ()
      ENDIF (EXISTS "${SPATH}/${input_file}.ice")
    ENDFOREACH (SPATH)
    IF (${SLICE_FILE_FOUND} STREQUAL "false")
      MESSAGE(FATAL_ERROR "${input_file}.ice not fund in any of the Slice directories (${SLICE_PATH}).")
#     ELSE (${SLICE_FILE_FOUND} STREQUAL "false")
#       MESSAGE( "${input_file}.ice was found!")
    ENDIF (${SLICE_FILE_FOUND} STREQUAL "false")
  ENDFOREACH( input_file )
ENDMACRO( ROBOCOMP_WRAP_ICE )

INCLUDE_DIRECTORIES (
  ${CMAKE_CURRENT_BINARY_DIR}
  .
  /opt/robocomp/include/
  ${ROBOCOMP_ROOT}/classes/
  ${CMAKE_BINARY_DIR}
)


MACRO( ROBOCOMP_WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    MESSAGE(STATUS "Adding rule to generate ui_${input_file}.py from ${input_file}.ui" )
    ADD_CUSTOM_COMMAND (
      OUTPUT ui_${input_file}.py
      COMMAND pyuic4 ${input_file}.ui -o ui_${input_file}.py
      DEPENDS ${input_file}.ui
      COMMENT "Generating ui_${input_file}.py from ${input_file}.ui"
    )
  ENDFOREACH( input_file )
ENDMACRO( ROBOCOMP_WRAP_PYTHON_UI )
