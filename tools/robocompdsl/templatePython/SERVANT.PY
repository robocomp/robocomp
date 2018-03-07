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

from parseCDSL import *
includeDirectories = theIDSLPaths.split('#')
component = CDSLParsing.fromFile(theCDSL, includeDirectories=includeDirectories)
if component == None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)

from parseIDSL import *
modulePool = IDSLPool(theIDSLs, includeDirectories)
module = modulePool.moduleProviding(theInterface)
if module == None:
	print('Can\'t locate', theIDSLs)
	sys.exit(1)


]]]
[[[end]]]
#
# Copyright (C)
[[[cog
A()
import datetime
cog.out(' '+str(datetime.date.today().year))
Z()
]]]
[[[end]]]
 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, Ice

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

additionalPathStr = ''
icePaths = []
try:
	icePaths.append('/opt/robocomp/interfaces')
	SLICE_PATH = os.environ['SLICE_PATH'].split(':')
	for p in SLICE_PATH:
		icePaths.append(p)
		additionalPathStr += ' -I' + p + ' '
except:
	print 'SLICE_PATH environment variable was not exported. Using only the default paths'
	pass

[[[cog
for incl1 in theIDSLs.strip().split('#'):
	incl = incl1.split('.')[0]
	if len(incl) == 0:
		continue
	# print('SERVANT <'+incl+'>')
	cog.outl('ice_'+incl+' = False')
	cog.outl('for p in icePaths:')
	# cog.outl('<TABHERE>print \'Trying\', p, \'to load ' + incl + '.ice\'')
	cog.outl('<TABHERE>if os.path.isfile(p+\'/'+incl+'.ice\'):')
	# cog.outl('<TABHERE><TABHERE>print \'Using\', p, \'to load '+incl+'.ice\'')
	cog.outl('<TABHERE><TABHERE>preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+\'/\'')
	cog.outl('<TABHERE><TABHERE>wholeStr = preStr+"'+incl+'.ice"')
	cog.outl('<TABHERE><TABHERE>Ice.loadSlice(wholeStr)')
	cog.outl('<TABHERE><TABHERE>ice_'+incl+' = True')
	cog.outl('<TABHERE><TABHERE>break')
	cog.outl('if not ice_'+incl+':')
	cog.outl("<TABHERE>print 'Couldn\\\'t load "+incl+"'")
	cog.outl('<TABHERE>sys.exit(-1)')
	module2 = IDSLParsing.gimmeIDSL(incl+'.idsl', files='', includeDirectories=includeDirectories)
	cog.outl('from '+ module2['name'] +' import *')

cog.outl('\nclass ' + theInterface+ 'I('+theInterface+'):')
]]]
[[[end]]]
	def __init__(self, worker):
		self.worker = worker

[[[cog

	for interface in module['interfaces']:
		if interface['name'] == theInterface:
			for mname in interface['methods']:
				method = interface['methods'][mname]

				name = method['name']

				paramStrA = ''
				for p in method['params']:
					if p['decorator'] != 'out':
						paramStrA += p['name'] + ', '
				paramStrB = ''
				for p in method['params']:
					if p['decorator'] != 'out':
						if paramStrB == '': delim = ''
						else: delim = ', '
						paramStrB += delim + p['name']


				cog.outl('<TABHERE>def ' + name + '(self, ' + paramStrA + 'c):')
				cog.out('<TABHERE><TABHERE>return self.worker.'+name+'(')
				cog.out(paramStrB)
				cog.out(')\n')

]]]
[[[end]]]
