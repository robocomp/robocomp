MESSAGE(STATUS "FrameWave will be used")
INCLUDE_DIRECTORIES( /usr/local/include /usr/include )
SET( LIBS ${LIBS} -L/usr/local/lib -lfwBase -lfwImage -lfwJPEG -lfwSignal -lfwVideo )
