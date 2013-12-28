#just to avoid the warning
if(COMMAND cmake_policy)
     cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
FIND_PACKAGE( OpenCV REQUIRED )

#set the path of opentldComp
SET(	OPENTLD_PATH 		$ENV{ROBOCOMP}/Components/RoboLab/Experimental/openTLDComp/OpenTLD)
MESSAGE(STATUS ${OPENTLD_PATH})

#set the default path for built libraries to the "lib" directory
SET(	LIBRARY_OUTPUT_PATH 	${OPENTLD_PATH}/lib)
MESSAGE(STATUS ${LIBRARY_OUTPUT_PATH})

#set the include directories
INCLUDE_DIRECTORIES (${OPENTLD_PATH}/include)
MESSAGE(STATUS ${LIBRARY_OUTPUT_PATH})

LINK_DIRECTORIES (${OPENTLD_PATH}/OpenTLD/lib/)

#libraries
add_library(	tld_utils 	${OPENTLD_PATH}/src/tld_utils.cpp		)
add_library(	LKTracker 	${OPENTLD_PATH}/src/LKTracker.cpp		)
add_library(	ferNN 		${OPENTLD_PATH}/src/FerNNClassifier.cpp	)
add_library(	tld 		${OPENTLD_PATH}/src/TLD.cpp				)

#set opentld libs
SET(	LIBS ${LIBS}		tld LKTracker ferNN tld_utils)


