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


from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communicationIsIce

includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)

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
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/qt.cmake )
INCLUDE ( CMakeListsSpecific.txt)

# Sources set
SET ( SOURCES
  ${SOURCES}
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/sigwatch/sigwatch.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
  main.cpp
  genericmonitor.cpp
  commonbehaviorI.cpp
  genericworker.cpp
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
	cog.outl('include_directories(  "/opt/ros/melodic/include" )')
	cog.outl('SET ( LIBS ${LIBS} ${catkin_LIBRARIES} -L/opt/ros/melodic/lib -lroscpp -lrosconsole -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lrosconsole_backend_interface)')
]]]
[[[end]]]

[[[cog
if component['language'].lower() == "cpp11":
	cog.outl('ADD_DEFINITIONS ("-DICE_CPP11_MAPPING")')
	cog.outl('FIND_PACKAGE( Ice REQUIRED COMPONENTS Ice++11 IceStorm++11)')
]]]
[[[end]]]


#ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior 
[[[cog
A()
imp = []
for im in component['recursiveImports'] + component["iceInterfaces"]:
    name = im.split('/')[-1].split('.')[0]
    imp.append(name)
cog.out(' '.join(imp))
try:
	options = [ x.lower() for x in component['options'] ]
	if 'agmagent' in options:
		cog.out(" AGMExecutive AGMWorldModel Planning AGMCommonBehavior ")
	elif 'agm2agent' in options or 'agm2agentICE' in options or 'agm2agentROS' in options:
		cog.out(" AGM2 ")
except:
	pass
Z()
]]]
[[[end]]]
)
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

[[[cog
	if component['gui'] is not None:
		cog.outl("QT_WRAP_UI( UI_HEADERS mainUI.ui )")
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
${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS} ${QT_LIBRARIES} ${Ice_LIBRARIES})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/
[[[cog
A()
cog.out(component['name'] + ' ')
Z()
]]]
[[[end]]]
DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
