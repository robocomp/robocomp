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

import templateCPP.functions.src.cmakelists_txt as cmakelists
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, is_agm1_agent, is_agm2_agent

includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)

]]]
[[[end]]]
cmake_minimum_required(VERSION 2.8.11)


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
  cog.out(cmakelists.interface_sources(component))
  cog.out(cmakelists.statemachine_visual_sources(component.statemachine_visual), trimblanklines=True)
  ]]]
  [[[end]]]
)

[[[cog
cog.out(cmakelists.ros_includes(component.usingROS), trimblanklines=True)
]]]
[[[end]]]

[[[cog
cog.out(cmakelists.cpp11_ice_packages(component.language))
]]]
[[[end]]]

[[[cog
cog.out(cmakelists.agm_includes(component), trimblanklines=True)
]]]
[[[end]]]

#ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
[[[cog
cog.out(cmakelists.wrap_ice(component))
]]]
[[[end]]]
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

[[[cog
if component.gui is not None:
    cog.outl("QT_WRAP_UI( UI_HEADERS mainUI.ui )")
]]]
[[[end]]]

# Specify construction and link process
[[[cog
cog.outl("ADD_EXECUTABLE( "+component.name +" ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )")
cog.outl("TARGET_LINK_LIBRARIES( "+ component.name +" ${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS} ${QT_LIBRARIES} ${Ice_LIBRARIES})")
cog.outl("INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/"+component.name +" DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )")
]]]
[[[end]]]