# FIND_PACKAGE( OpenCV REQUIRED )
# INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIR} )
# SET(LIBS ${LIBS} ${OpenCV_LIBS})
# MESSAGE(STATUS ${OpenCV_LIBS})
# 
# 
INCLUDE_DIRECTORIES( /usr/lib/x86_64-linux-gnu )

execute_process(COMMAND pkg-config --libs opencv OUTPUT_VARIABLE rv OUTPUT_STRIP_TRAILING_WHITESPACE)
message("rv = '@@@${rv}'@@@")

STRING(REGEX REPLACE "-lippicv" "" rv ${rv})

message("rv = '@@@${rv}'@@@")


SET(LIBS ${LIBS} "-L/usr/lib/x86_64-linux-gnu-L/usr/lib ${rv}")
