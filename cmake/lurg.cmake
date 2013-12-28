IF( "$ENV{URGROOT}" STREQUAL "" )
  MESSAGE(STATUS "TODO: Try to automatically find l_urg library." )
ENDIF( "$ENV{URGROOT}" STREQUAL "" )


IF( "$ENV{URGROOT}" STREQUAL "" )
  MESSAGE(STATUS "URGROOT environment variable not set and we couln't found automatically." )
  MESSAGE(STATUS "This can be done in your user .bashrc file by appending the corresponding line, e.g:" )
  MESSAGE(STATUS "export URGROOT=/usr/local/" )
  SET(LURG_FOUND 0)
ELSE( "$ENV{URGROOT}" STREQUAL "" )
  SET(URG_LIBS -L$ENV{URGROOT}/lib/ -lc_urg -lc_urg_system -lc_urg_connection )
  INCLUDE_DIRECTORIES( "$ENV{URGROOT}/include/")
  SET( LIBS ${LIBS} ${URG_LIBS} -L$ENV{URGROOT}/lib/ )
  ADD_DEFINITIONS(-DCOMPILE_HOKUYO30LX=1)
  SET(LURG_FOUND 1)
  SET (LIBS ${LIBS} ${URG_LIBS})
ENDIF( "$ENV{URGROOT}" STREQUAL "" )

