FIND_PACKAGE( OpenCV 2 )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIR} )
SET(LIBS ${LIBS} ${OpenCV_LIBS})
MESSAGE(STATUS ${OpenCV_LIBS})

# INCLUDE_DIRECTORIES( /usr/local/include/opencv /usr/include/opencv )
# SET( LIBS ${LIBS} -L/usr/local/lib -L/usr/lib -lcv -lhighgui -lcvaux -lml -lcxcore )
