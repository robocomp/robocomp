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



MACRO( WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    ADD_CUSTOM_COMMAND (
      OUTPUT src/ui_${input_file}.py
      COMMAND pyside-uic src/${input_file}.ui -o src/ui_${input_file}.py
      DEPENDS src/${input_file}.ui
      COMMENT "Generating src/ui_${input_file}.py from src/${input_file}.ui"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS src/ui_${input_file}.py )
  ENDFOREACH( input_file )
ENDMACRO( WRAP_PYTHON_UI )

[[[cog
A()
if component['gui'] != 'none':
	cog.out('WRAP_PYTHON_UI( mainUI )')
Z()
]]]
[[[end]]]

