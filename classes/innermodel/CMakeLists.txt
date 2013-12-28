cmake_minimum_required( VERSION 2.8 )

INCLUDE_DIRECTORIES( . .. )
# ADD_DEFINITIONS( -g -O3 -Wall -Werror )
ADD_DEFINITIONS( -g -O3 -Wall )


# Source files
SET ( SOURCES
	im1/innermodel.cpp
	im1/innermodelreader.cpp
	im1/innermodelviewer.cpp
)

# IPP
INCLUDE( $ENV{ROBOCOMP}/cmake/ipp.cmake )

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )

# Qt
ADD_DEFINITIONS( -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB -DQT_XML_LIB -DQT_XML )
FIND_PACKAGE( Qt4 COMPONENTS QtCore QtGui QtDBus QtXml QtXmlPatterns REQUIRED)
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTXMLPATTERNS TRUE )
SET( QT_USE_QTSTATE TRUE )
SET( QT_USE_QTSTATEMACHINE TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )

# Bullet
IF( BUILD_PHYSICS )
	FIND_PACKAGE( Bullet REQUIRED )
	ADD_DEFINITIONS( -DUSE_BULLET )
	INCLUDE_DIRECTORIES( ${BULLET_INCLUDE_DIR} )
	SET( SOURCES ${SOURCES}
		im2/innermodel.cpp
		im2/joints/BallJoint.cpp
		im2/joints/GenericJoint.cpp
		im2/joints/HingeJoint.cpp
		im2/joints/Joint.cpp
		im2/joints/PrismaticJoint.cpp
		im2/nodes/Body.cpp
		im2/nodes/DifferentialRobot.cpp
		im2/nodes/Node.cpp
		im2/primitives/Mesh.cpp
		im2/primitives/Plane.cpp
		im2/primitives/PointCloud.cpp
		im2/primitives/Primitive.cpp
		im2/sensors/Camera.cpp
		im2/sensors/IMU.cpp
		im2/sensors/Laser.cpp
		im2/sensors/RGBD.cpp
		im2/sensors/Sensor.cpp
	)
	SET( LIBS ${LIBS} ${BULLET_LIBRARIES} osgbCollision )
ENDIF()

# Documentation
#FIND_PACKAGE( Doxygen )
#IF( DOXYGEN_FOUND )
#	CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY )
#	ADD_CUSTOM_TARGET( doc ALL
#		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
#		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#		COMMENT "Generating API documentation with Doxygen" VERBATIM )
#ENDIF()

# OPTION( BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ON )
# IF(BUILD_DOCUMENTATION)
# 	FIND_PACKAGE(Doxygen)
# 	IF(NOT DOXYGEN_FOUND)
# 		MESSAGE(FATAL_ERROR
# 		"Doxygen is needed to build the documentation.")
# 	ENDIF()
# 	
# 	SET( doxyfile_in          ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in     )
# 	SET( doxyfile             ${PROJECT_BINARY_DIR}/Doxyfile              )
# 	SET( doxy_html_index_file ${CMAKE_CURRENT_BINARY_DIR}/html/index.html )
# 	SET( doxy_output_root     ${CMAKE_CURRENT_BINARY_DIR}                 ) # Pasted into Doxyfile.in
# 	SET( doxy_input           ${PROJECT_SOURCE_DIR}/src                   ) # Pasted into Doxyfile.in
# # 	SET( doxy_extra_files     ${CMAKE_CURRENT_SOURCE_DIR}/mainpage.dox    ) # Pasted into Doxyfile.in
# 	
# 	CONFIGURE_FILE( ${doxyfile_in} ${doxyfile} @ONLY )
# 	
# 	ADD_CUSTOM_COMMAND( OUTPUT ${doxy_html_index_file}
# 						COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
# 						# The following should be ${doxyfile} only but it
# 						# will break the dependency.
# 						# The optimal solution would be creating a 
# 						# custom_command for ${doxyfile} generation
# 						# but I still have to figure out how...
# 						MAIN_DEPENDENCY ${doxyfile} ${doxyfile_in}
# 						DEPENDS ${SOURCES} ${doxy_extra_files}
# 						COMMENT "Generating HTML documentation")
# 	
# 	ADD_CUSTOM_TARGET( doc ALL DEPENDS ${doxy_html_index_file} )
# 	
# 	INSTALL( DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION /opt/robocomp/share/doc )
# ENDIF()

# Construction and link process
ADD_LIBRARY ( innermodel SHARED ${SOURCES} ${MOC_SOURCES} )
TARGET_LINK_LIBRARIES ( innermodel ${LIBS} ${QT_LIBRARIES} )
INSTALL ( FILES ${CMAKE_CURRENT_BINARY_DIR}/libinnermodel.so DESTINATION /opt/robocomp/lib/ )
INSTALL ( FILES innermodel.h DESTINATION /opt/robocomp/include/innermodel )
INSTALL ( FILES im1/innermodel.h im1/innermodelreader.h im1/innermodelviewer.h DESTINATION /opt/robocomp/include/innermodel/im1/ )
INSTALL ( FILES im2/innermodel.h DESTINATION /opt/robocomp/include/innermodel/im2/ )
INSTALL ( FILES im2/InnerModel2.xsd DESTINATION /opt/robocomp/share/ )
