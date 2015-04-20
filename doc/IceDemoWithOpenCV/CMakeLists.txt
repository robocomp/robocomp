cmake_minimum_required(VERSION 2.8)
find_package( OpenCV REQUIRED )
SET( ICE_HOME /usr)
SET( SOURCE_FILES Printer.cpp server.cpp)
SET( SOURCE_FILES1 Printer.cpp client.cpp)

include_directories(
	${ICE_HOME}/include/
)

add_executable (server ${SOURCE_FILES})
add_executable (client ${SOURCE_FILES1})

TARGET_LINK_LIBRARIES(${OpenCV_CFLAGS} server 
	Ice
	IceUtil
	pthread
	${ICE_HOME}/lib/
	${OpenCV_LIBS}
	
)

TARGET_LINK_LIBRARIES(${OpenCV_CFLAGS} client 
	Ice
	IceUtil
	pthread
	${ICE_HOME}/lib/
	${OpenCV_LIBS}
	
)
	

