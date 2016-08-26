[[[cog

import sys
sys.path.append('/opt/robocomp/python')

import cog
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
component = CDSLParsing.fromFile(theCDSL)

]]]
[[[end]]]
cmake_minimum_required(VERSION 2.8.11)
PROJECT( 
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
	if communicationIsIce(ima):
		cog.outl('<S2>' + im.lower() + 'I.cpp')

for subscribe in component['subscribesTo']:
	subs = subscribe
	while type(subs) != type(''):
		subs = subs[0]
	if communicationIsIce(subscribe):
		cog.outl('<S2>' + subs.lower() + 'I.cpp')
		

]]]
[[[end]]]
)

[[[cog
if component['usingROS'] == True:
	cog.outl('#ROS')
	cog.outl('find_package(catkin REQUIRED)')
	cog.outl('include_directories(  ${catkin_INCLUDE_DIRS} )')
	cog.outl('include_directories(  "/opt/ros/kinetic/include" )')
	cog.outl('SET ( LIBS ${LIBS} ${catkin_LIBRARIES} -L/opt/ros/kinetic/lib -lroscpp -lrosconsole -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lrosconsole_backend_interface)')
]]]
[[[end]]]


#ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior 
[[[cog
A()
for im in component['recursiveImports']:
	if communicationIsIce(im):
		cog.out(im.split('/')[-1].split('.')[0] + ' ')
try:
	options = [ x.lower() for x in component['options'] ]
	if 'agmagent' in options:
		cog.out("AGMExecutive AGMWorldModel Planning AGMCommonBehavior ")
	elif 'agm2agent' in options or 'agm2agentICE' in options or 'agm2agentROS' in options:
		cog.out("AGM2 ")
except:
	pass
Z()
]]]
[[[end]]]
)
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

[[[cog
if component['useQt'] == 'qt5':
	cog.outl("# Qt5")
	cog.outl("ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB -fPIC)")
	cog.outl("FIND_PACKAGE(Qt5Core REQUIRED)")
	cog.outl("FIND_PACKAGE(Qt5Gui REQUIRED)")
	cog.outl("FIND_PACKAGE(Qt5Xml REQUIRED)")
	cog.outl("FIND_PACKAGE(Qt5Widgets REQUIRED)")
	cog.outl("find_package(Qt5OpenGL REQUIRED)")
	cog.outl("message(STATUS, ${Qt5Widgets_INCLUDE_DIRS})")
	cog.outl("include_directories(${Qt5Widgets_INCLUDE_DIRS})")
	cog.outl("include_directories(${Qt5Core_INCLUDE_DIRS})")
	cog.outl("include_directories(${Qt5_INCLUDE_DIRS})")
	cog.outl("SET (LIBS ${LIBS} ${Qt5Widgets_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5_LIBRARIES})" )
	if component['gui'] != 'none':
		cog.outl("QT5_WRAP_UI( UI_HEADERS mainUI.ui )")
	cog.outl("SET( CMAKE_AUTOMOC TRUE )")
else:
	cog.outl("# Qt4")
	cog.outl("ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )")
	cog.outl("FIND_PACKAGE( Qt4 REQUIRED )")
	cog.outl("SET( QT_USE_QTGUI TRUE )")
	cog.outl("SET( QT_USE_QTOPENGL TRUE )")
	cog.outl("SET( QT_USE_QTXML TRUE )")
	cog.outl("SET( QT_USE_QTSTATE TRUE )")
	cog.outl("SET( QT_USE_QTSTATEMACHINE TRUE )")
	cog.outl("SET( CMAKE_AUTOMOC TRUE )")
	cog.outl("INCLUDE( ${QT_USE_FILE} )")
	if component['gui'] != 'none':
		cog.outl("QT4_WRAP_UI( UI_HEADERS mainUI.ui )")
]]]
[[[end]]]

# Specify construction and link process
[[[cog

try:
	options = [ x.lower() for x in component['options'] ]
	if 'agmagent' in options:
		cog.outl("SET(LIBS ${LIBS} -lagm)")
	elif 'agm2agent' in options or  'agm2agentICE' in options or  'agm2agentROS' in options:
		cog.outl("SET(LIBS ${LIBS} -lagm2)")

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
if not component['useQt'] == 'qt5':
	cog.out('${QT_LIBRARIES} ')
Z()
]]]
[[[end]]]
${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/
[[[cog
A()
cog.out(component['name'] + ' ')
Z()
]]]
[[[end]]]
DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
