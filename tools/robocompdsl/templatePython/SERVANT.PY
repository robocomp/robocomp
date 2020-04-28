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

from templatePython.functions import servant_py as servant
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import IDSLPool
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
if component == None:
    raise ValueError('SERVANT.PY: Can\'t locate %s' % theCDSLs)

modulePool = IDSLPool(theIDSLs, includeDirectories)
module = modulePool.moduleProviding(theInterface)
if module == None:
    raise ValueError('SERVANT.PY: Can\'t locate %s' % theIDSLs)


]]]
[[[end]]]
#
[[[cog
import datetime
cog.out('# Copyright (C) '+str(datetime.date.today().year)+' by YOUR NAME HERE')
]]]
[[[end]]]
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
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
    raise RuntimeError('ROBOCOMP environment variable not set! Exiting.')


additionalPathStr = ''
icePaths = []
try:
    icePaths.append('/opt/robocomp/interfaces')
    SLICE_PATH = os.environ['SLICE_PATH'].split(':')
    for p in SLICE_PATH:
        icePaths.append(p)
        additionalPathStr += ' -I' + p + ' '
except:
    print('SLICE_PATH environment variable was not exported. Using only the default paths')
    pass
[[[cog
cog.out(servant.slice_loading(theInterface), trimblanklines=False)
module2 = DSLFactory().from_file(theInterface+'.idsl', includeDirectories=includeDirectories)
cog.outl('from '+ module2['name'] +' import *')
cog.outl('\nclass ' + theInterface+ 'I('+theInterface+'):')
]]]
[[[end]]]
    def __init__(self, worker):
        self.worker = worker

    [[[cog
    cog.out(servant.interface_methods(module, theInterface),  trimblanklines=True)
    ]]]
    [[[end]]]
