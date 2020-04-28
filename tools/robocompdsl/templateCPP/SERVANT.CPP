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

from templateCPP.functions import servant
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import decorator_and_type_to_const_ampersand, IDSLPool
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
if component == None:
	raise ValueError('SERVANT.CPP Can\'t locate %s ' % theCDSLs)

modulePool = IDSLPool(theIDSLs, includeDirectories)

theInterface = theInterface.split(';')
module = modulePool.moduleProviding(theInterface[0])
if module == None:
	raise ValueError('SERVANT.CPP: Can\'t locate %s' % theIDSLs)


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
[[[cog
cog.out("#include \""+theInterface[0].lower()+"I.h\"")
]]]
[[[end]]]

[[[cog
cog.out(theInterface[0]+"I::"+theInterface[0]+"I(GenericWorker *_worker)")
]]]
[[[end]]]
{
	worker = _worker;
}


[[[cog
cog.out(theInterface[0]+"I::~"+theInterface[0]+"I()")
]]]
[[[end]]]
{
}

[[[cog
cog.out(servant.interface_methods_creation(component, modulePool, theInterface), trimblanklines=True)
]]]
[[[end]]]

