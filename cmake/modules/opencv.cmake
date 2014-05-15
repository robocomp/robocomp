# FIND_PACKAGE( OpenCV REQUIRED )
# INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIR} )
# SET(LIBS ${LIBS} ${OpenCV_LIBS})
# MESSAGE(STATUS ${OpenCV_LIBS})
# 
# 
INCLUDE_DIRECTORIES( /usr/lib/x86_64-linux-gnu )
SET( LIBS ${LIBS} -L/usr/lib/x86_64-linux-gnu-L/usr/lib -lopencv_core -lopencv_highgui -lopencv_ml -lopencv_imgproc -lopencv_calib3d -lopencv_contrib )
