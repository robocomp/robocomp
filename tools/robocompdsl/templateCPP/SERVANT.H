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

import templateCPP.functions.servant_h as servant
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import decorator_and_type_to_const_ampersand,IDSLPool
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
if component == None:
    raise ValueError('SERVANT.H: Can\'t locate %s' % theCDSLs)

modulePool = IDSLPool(theIDSLs, includeDirectories)

theInterface = theInterface.split(';')
while type(theInterface) != type(''):
	theInterface = theInterface[0]

module = modulePool.moduleProviding(theInterface)
if module == None:
	raise ValueError('SERVANT.H: Can\'t locate %s' % theInterface)
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
[[[cog
cog.outl('#ifndef ' + theInterface.upper()+'_H')
cog.outl('#define ' + theInterface.upper()+'_H')
]]]
[[[end]]]

// Ice includes
#include <Ice/Ice.h>
[[[cog
cog.out('#include <'+module['filename'].split('/')[-1].split('.')[0]+'.h>')
]]]
[[[end]]]

#include <config.h>
#include "genericworker.h"

[[[cog
cog.out('using namespace ' + module['name']+';')
]]]
[[[end]]]

[[[cog
cog.out('class ' + theInterface+'I : public virtual ' + module['name']+'::'+theInterface)
]]]
[[[end]]]
{
public:
	[[[cog
	cog.outl(theInterface+'I(GenericWorker *_worker);')
	cog.outl('~'+theInterface+'I();')
	]]]
	[[[end]]]

	[[[cog
	cog.out(servant.interface_methods_definition(component, module, modulePool, theInterface))
	]]]
	[[[end]]]

private:

	GenericWorker *worker;

};

#endif
