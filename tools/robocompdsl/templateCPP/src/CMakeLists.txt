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

cmake_minimum_required(VERSION 2.6)
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
if component['usingROS']:
	cog.outl('#ROS')
	cog.outl('find_package(catkin REQUIRED)')
	cog.outl('include_directories(  ${catkin_INCLUDE_DIRS} )')
	cog.outl('SET ( LIBS ${LIBS} ${catkin_LIBRARIES} -L/opt/ros/jade/lib -lroscpp -lrosconsole -lroscpp_serialization)')
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
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("SET(LIBS ${LIBS} -lagm)")


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
