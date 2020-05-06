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

from dsl_parsers.dsl_factory import DSLFactory
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)

]]]
[[[end]]]
#ifndef CONFIG_H
#define CONFIG_H

// Comment out this line if your application has a QtGui
[[[cog
if component.gui is not None:
	cog.outl("#define USE_QTGUI\n")
]]]
[[[end]]]

#define PROGRAM_NAME    "
[[[cog
A()
cog.out(component.name)
Z()
]]]
[[[end]]]
"
#define SERVER_FULL_NAME   "RoboComp
[[[cog
A()
cog.out(' ' + component.name)
Z()
]]]
[[[end]]]
::
[[[cog
A()
cog.out(' ' + component.name)
Z()
]]]
[[[end]]]
"

#endif
