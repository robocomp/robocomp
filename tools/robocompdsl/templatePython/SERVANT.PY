[[[cog

import sys
from robocomp_general import config_robocomp
config_information = config_robocomp("/opt/robocomp/share/robocompdsl/robocompdsl_config.json").config

import cog
def A():
	cog.out('<@@<')
def Z():
	cog.out('>@@>')
def TAB():
	cog.out('<TABHERE>')

from parseCDSL import *
component = CDSLParsing.fromFile(theCDSL)
if component == None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)

from parseIDSL import *
modulePool = IDSLPool(theIDSLs)
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

[[[cog
cog.outl('from robocomp_general import config_robocomp')
cog.outl('config_information = config_robocomp("/opt/robocomp/share/robocompdsl/robocompdsl_config.json").config')
]]]
[[[end]]]

if config_information["pathfiles"]["path2localrobocomp"] == "":
	print 'path2localrobocomp in config file not set, using the default value /opt/robocomp'
	config_information["pathfiles"]["path2localrobocomp"] = '/opt/robocomp'
if len(config_information["pathfiles"]["path2localrobocomp"]) == "":
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

[[[cog
cog.outl('preStr = "-I"+config_information["pathfiles"]["path2localrobocomp"]+"/interfaces/ --all "+config_information["pathfiles"]["path2localrobocomp"]+"/interfaces/"')
cog.outl('\nIce.loadSlice(preStr+"'+theInterface+'.ice")')
cog.outl('from ' + module['name'] + ' import *')
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





