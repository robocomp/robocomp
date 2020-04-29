#!/usr/bin/python3
# -*- coding: utf-8 -*-
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

from templatePython.functions import specificworker_py as specificworker
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool

includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
sm = DSLFactory().from_file(component.statemachine)
if component == None:
    raise ValueError("specificworker.py: Can\'t locate %s" % theCDSLs)



pool = IDSLPool(theIDSLs, includeDirectories)

def replaceTypeCPP2Python(t):
    t = t.replace('::','.')
    t = t.replace('string', 'str')
    return t

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

from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        [[[cog
        if sm is None:
            cog.outl("self.timer.timeout.connect(self.compute)")
        ]]]
        [[[end]]]
        self.Period = 2000
        self.timer.start(self.Period)

        [[[cog
        if sm is not None:
            cog.outl("self." + sm['machine']['name'] + ".start()")
            if sm['machine']['default']:
                cog.outl("self.destroyed.connect(self.t_compute_to_finalize)")
        ]]]
        [[[end]]]

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        #try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        #except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    [[[cog
    cog.out(specificworker.compute_creation(component, sm))
    ]]]
    [[[end]]]

    [[[cog
    cog.out(specificworker.statemachine_slots(sm))
    ]]]
    [[[end]]]
    [[[cog
    cog.out(specificworker.subscription_methods(component, pool))
    ]]]
    [[[end]]]

    [[[cog
    cog.out(specificworker.implements_methods(component, pool))
    ]]]
    [[[end]]]
