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
cmake_minimum_required( VERSION 2.8 )

SUBDIRS( src )

INSTALL(FILES etc/config DESTINATION /opt/robocomp/etc-default/ RENAME
[[[cog
A()
cog.out(' ' + component['name'])
Z()
]]]
[[[end]]]
.conf )

ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
ADD_CUSTOM_TARGET(installdoc mkdir -p /opt/robocomp/doc COMMAND cp -R doc/html /opt/robocomp/doc/
[[[cog
A()
cog.out( component['name'] + ' ')
Z()
]]]
[[[end]]]
)