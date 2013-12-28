IF( "$ENV{SICKROOT}" STREQUAL "" )
  MESSAGE(STATUS "TODO: Try to automatically find l_sick library." )
ENDIF( "$ENV{SICKROOT}" STREQUAL "" )


IF( "$ENV{SICKROOT}" STREQUAL "" )
  MESSAGE(STATUS "SICKROOT environment variable not set and we couln't found automatically." )
  MESSAGE(STATUS "This can be done in your user .bashrc file by appending the corresponding line, e.g:" )
  MESSAGE(STATUS "export SICKROOT=/usr/local" )
  SET(LSICK_FOUND 0)
ELSE( "$ENV{SICKROOT}" STREQUAL "" )
  SET(SICK_LIBS -L/usr/local/lib/ -lsicklms-1.0 -lsickld-1.0  )
  INCLUDE_DIRECTORIES( "$ENV{SICKROOT}/include/")
  SET( LIBS ${LIBS} ${SICK_LIBS} -L$ENV{SICKROOT}/lib/ )
  ADD_DEFINITIONS(-DCOMPILE_SICK=1)
  SET(LSICK_FOUND 1)
  SET (LIBS ${LIBS} ${SICK_LIBS})
ENDIF( "$ENV{SICKROOT}" STREQUAL "" )

