/*
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
 *    Copyright (C) 
[[[cog
A()
import datetime
cog.out(str(datetime.date.today().year))
Z()
]]]
[[[end]]]
 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "
[[[cog
A()
cog.out(theInterface.lower())
Z()
]]]
[[[end]]]
I.h"

[[[cog
cog.out(theInterface)
Z()
]]]
[[[end]]]
I::
[[[cog
A()
cog.out(theInterface)
Z()
]]]
[[[end]]]
I(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


[[[cog
cog.out(theInterface)
Z()
]]]
[[[end]]]
I::~
[[[cog
A()
cog.out(theInterface)
Z()
]]]
[[[end]]]
I()
{
}

[[[cog

	for interface in module['interfaces']:
		if interface['name'] == theInterface:
			for mname in interface['methods']:
				method = interface['methods'][mname]
				
				ret = method['return']
				name = method['name']
				
				paramStrA = ''
				for p in method['params']:
					if p['decorator'] == 'out':
						const = ''
					else:
						const = 'const '
					if  p['type'] in [ 'int' ]:
						ampersand = ''
					else:
						ampersand = '&'
					paramStrA += const + p['type'] + ' ' + ampersand + p['name'] + ', '

					
				paramStrB = ''
				for p in method['params']:
					if paramStrB == '': delim = ''
					else: delim = ', '
					paramStrB += delim + p['name']

				cog.outl(ret + ' ' + theInterface + 'I::' + name + '(' + paramStrA + 'const Ice::Current&)')
				cog.outl('{')
				cog.out('<TABHERE>worker->'+name+'(')
				cog.out(paramStrB)
				cog.out(');\n')
				cog.outl('}\n')

]]]
[[[end]]]





