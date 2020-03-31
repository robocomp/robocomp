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

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import decorator_and_type_to_const_ampersand, IDSLPool
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
if component == None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)

modulePool = IDSLPool(theIDSLs, includeDirectories)

theInterface = theInterface.split(';')
module = modulePool.moduleProviding(theInterface[0])
if module == None:
	print('Can\'t locate', theIDSLs)
	sys.exit(1)


]]]
[[[end]]]
 *    Copyright (C)
[[[cog
A()
import datetime
cog.out(' '+str(datetime.date.today().year))
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
cog.out(theInterface[0].lower())
Z()
]]]
[[[end]]]
I.h"

[[[cog
cog.out(theInterface[0])
Z()
]]]
[[[end]]]
I::
[[[cog
A()
cog.out(theInterface[0])
Z()
]]]
[[[end]]]
I(GenericWorker *_worker)
{
	worker = _worker;
}


[[[cog
cog.out(theInterface[0])
Z()
]]]
[[[end]]]
I::~
[[[cog
A()
cog.out(theInterface[0])
Z()
]]]
[[[end]]]
I()
{
}

[[[cog

	for interface in module['interfaces']:
		if interface['name'] == theInterface[0]:
			for mname in interface['methods']:
				method = interface['methods'][mname]

				ret = method['return']
				name = method['name']

				paramStrA = ''
				for p in method['params']:
					const, ampersand = decorator_and_type_to_const_ampersand(p['decorator'], p['type'], modulePool, component['language'].lower() == "cpp11")
					if p['type'] == 'long':
						paramStrA += const + 'Ice::Long' + ' ' + ampersand + p['name'] + ', '
					else:
						paramStrA += const + p['type'] + ' ' + ampersand + p['name'] + ', '

				paramStrB = ''
				for p in method['params']:
					if paramStrB == '': delim = ''
					else: delim = ', '
					paramStrB += delim + p['name']

				cog.outl(ret + ' ' + theInterface[0] + 'I::' + name + '(' + paramStrA + 'const Ice::Current&)')
				cog.outl('{')
				cog.out('<TABHERE>')
				if ret != 'void':
					cog.out('return ')
				cog.out('worker->'+interface['name']+"_"+name+'(')
				cog.out(paramStrB)
				cog.out(');\n')
				cog.outl('}\n')

]]]
[[[end]]]
