MESSAGE(STATUS "TODO: Try to automatically find qmat library." )

SET (input_library qmat)

IF( EXISTS "/opt/robocomp/lib/lib${input_library}.so") 
	MESSAGE(STATUS "Adding library ${input_library} " )
	SET( LIBS ${LIBS} -L/opt/robocomp/lib/ -l${input_library} )
	INCLUDE_DIRECTORIES(   /opt/robocomp/include/ )
ELSE( EXISTS "/opt/robocomp/lib/lib${input_library}.so")
	MESSAGE(SEND_ERROR "Library ${input_library} not found in /opt/robocomp/lib" )
	MESSAGE(STATUS "Go to robocomp/Classes/qmatrix and type cmake . && make && make install to install it" )
ENDIF( EXISTS "/opt/robocomp/lib/lib${input_library}.so")

