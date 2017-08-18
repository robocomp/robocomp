[[[cog
import sys
sys.path.append('/opt/robocomp/python')

import cog

includeDirectories = theIDSLPaths.split('#')







def A():
	cog.out('<@@<')
def Z():
	cog.out('>@@>')
def TAB():
	cog.out('<TABHERE>')
def SPACE(i=0):
	s = ''
	if i>0:
		s = str(i)
	cog.out('<S'+s+'>')


from parseCDSL import *


includeDirectories = theIDSLPaths.split('#')
component = CDSLParsing.fromFile(theCDSL, includeDirectories=includeDirectories)

]]]
[[[end]]]
<<<<<<< HEAD
cmake_minimum_required(VERSION 2.8.11)
PROJECT(
=======

cmake_minimum_required(VERSION 2.6)
PROJECT( 
>>>>>>> master
[[[cog
A()
cog.out(' ' + component['name'])
Z()
]]]
[[[end]]]
 )


IF ( "$ENV{ROBOCOMP}" STREQUAL "")
  MESSAGE(WARNING "ROBOCOMP variable not set. Using the default value: /opt/robocomp")
  SET (ENV{ROBOCOMP} "/opt/robocomp/")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
  MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
INCLUDE ( CMakeListsSpecific.txt)

# Sources set
SET ( SOURCES
  ${SOURCES}
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/sigwatch/sigwatch.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
  main.cpp
  genericmonitor.cpp
  specificmonitor.cpp
  commonbehaviorI.cpp
  genericworker.cpp
  specificworker.cpp
[[[cog
for ima in component['implements']:
	if type(ima) == str:
		im = ima
	else:
		im = ima[0]
	cog.outl('<S2>' + im.lower() + 'I.cpp')

for subscribe in component['subscribesTo']:
	subs = subscribe
	while type(subs) != type(''):
		subs = subs[0]
	if communicationIsIce(subscribe):
		cog.outl('<S2>' + subs.lower() + 'I.cpp')
<<<<<<< HEAD

=======
	else:
		cog.outl('<S2>ROS' + subs.lower() + 'I.cpp')
		
>>>>>>> master

]]]
[[[end]]]
)

[[[cog

usingROS=False

for subscribe in component['subscribesTo']:
	subs = subscribe
	while type(subs) != type(''):
		subs = subs[0]
	if communicationIsIce(subscribe):
		pass
	else:
		usingROS = True

if usingROS:
	cog.outl('find_package(catkin REQUIRED)')
	cog.outl('include_directories(  ${catkin_INCLUDE_DIRS} )')
<<<<<<< HEAD
	cog.outl('include_directories(  "/opt/ros/kinetic/include" )')
	cog.outl('SET ( LIBS ${LIBS} ${catkin_LIBRARIES} -L/opt/ros/kinetic/lib -lroscpp -lrosconsole -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lrosconsole_backend_interface)')
=======
	cog.outl('SET ( LIBS ${LIBS} ${catkin_LIBRARIES} -L/opt/ros/jade/lib -lroscpp -lrosconsole -lroscpp_serialization) # -lrosbag')
>>>>>>> master
]]]
[[[end]]]


#ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior
[[[cog
A()
for im in component['recursiveImports']:
<<<<<<< HEAD
	if communicationIsIce(im):
		cog.out(' ' + im.split('/')[-1].split('.')[0] + ' ')
try:
	options = [ x.lower() for x in component['options'] ]
	if 'agmagent' in options:
		cog.out(" AGMExecutive AGMWorldModel Planning AGMCommonBehavior ")
	elif 'agm2agent' in options or 'agm2agentICE' in options or 'agm2agentROS' in options:
		cog.out(" AGM2 ")
except:
	pass
=======
	cog.out(im.split('/')[-1].split('.')[0] + ' ')
>>>>>>> master
Z()
]]]
[[[end]]]
)
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTSTATE TRUE )
SET( QT_USE_QTSTATEMACHINE TRUE )
SET( CMAKE_AUTOMOC TRUE )
INCLUDE( ${QT_USE_FILE} )

[[[cog
if component['gui'] != 'none':
	cog.outl("QT4_WRAP_UI( UI_HEADERS mainUI.ui )")
]]]
[[[end]]]



# Specify construction and link process
[[[cog

try:
	if isAGM1Agent(component):
		cog.outl("SET(LIBS ${LIBS} -lagm)")
		cog.outl("ADD_DEFINITIONS( -I/usr/include/libxml2/ )")
	if isAGM2Agent(component):
		cog.outl("SET(LIBS ${LIBS} -lagm2)")
		cog.outl("ADD_DEFINITIONS( -I/usr/include/libxml2/ )")
		cog.outl("include(/usr/local/share/cmake/FindAGM2.cmake)")
except:
	pass

]]]
[[[end]]]
ADD_EXECUTABLE(
[[[cog
A()
cog.out(' ' + component['name'] + ' ')
Z()
]]]
[[[end]]]
${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES(
[[[cog
A()
cog.out(' ' + component['name'] + ' ')
Z()
]]]
[[[end]]]
${QT_LIBRARIES} ${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/
[[[cog
A()
cog.out(component['name'] + ' ')
Z()
]]]
[[[end]]]
DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
