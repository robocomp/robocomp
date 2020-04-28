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
def SPACE(i=0):
    s = ''
    if i>0:
        s = str(i)
    cog.out('<S'+s+'>')

includeDirectories = theIDSLPaths.split('#')
from templatePython.functions import genericworker_py as genericworker
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import get_name_number, IDSLPool, communication_is_ice

component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)

if component == None:
    raise ValueError('genericworker.py: Can\'t locate %s' % theCDSL)

sm = DSLFactory().from_file(component.statemachine)


pool = IDSLPool(theIDSLs, includeDirectories)
modulesList = pool.rosModulesImports()

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

import sys, Ice, os
from PySide2 import QtWidgets, QtCore

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'

preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ --all /opt/robocomp/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior

additionalPathStr = ''
icePaths = [ '/opt/robocomp/interfaces' ]
try:
    SLICE_PATH = os.environ['SLICE_PATH'].split(':')
    for p in SLICE_PATH:
        icePaths.append(p)
        additionalPathStr += ' -I' + p + ' '
    icePaths.append('/opt/robocomp/interfaces')
except:
    print('SLICE_PATH environment variable was not exported. Using only the default paths')
    pass

[[[cog
cog.out(genericworker.load_slice_and_create_imports(component, includeDirectories), trimblanklines=True)
]]]
[[[end]]]

[[[cog
    for im in component.implements + component.subscribesTo:
        if communication_is_ice(im):
            name = im[0]
            cog.outl('from ' + name.lower() + 'I import *')
]]]
[[[end]]]

[[[cog
cog.out(genericworker.ros_imports(component, pool))
cog.out(genericworker.ui_import(component.gui))
]]]
[[[end]]]

[[[cog
cog.out(genericworker.ros_class_creation(component, pool))
]]]
[[[end]]]


[[[cog
if component.gui is not None:
    inherit_from = 'QtWidgets.'+component.gui[1]
else:
    inherit_from = 'QtCore.QObject'
cog.outl('class GenericWorker({}):'.format(inherit_from))
]]]
[[[end]]]
    [[[cog
    #if sm is not "none":
        #cog.outl("QtCore.__metaclass__  =  ABCMeta")
    ]]]
    [[[end]]]

    kill = QtCore.Signal()
    [[[cog
    cog.out(genericworker.statemachine_signals(sm))
    ]]]
    [[[end]]]

    def __init__(self, mprx):
        super(GenericWorker, self).__init__()


        [[[cog
        cog.out(genericworker.requires_proxies(component))
        cog.out(genericworker.publishes_proxies(component))
        ]]]
        [[[end]]]
        [[[cog
        cog.out(genericworker.gui_setup(component.gui), trimblanklines=True)
        ]]]
        [[[end]]]

        self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
        self.Period = 30
        self.timer = QtCore.QTimer(self)

        [[[cog
        cog.out(genericworker.statemachine_states_creation(sm))
        ]]]
        [[[end]]]

    [[[cog
    cog.out(genericworker.statemachine_slots_creation(sm))
    ]]]
    [[[end]]]
    @QtCore.Slot()
    def killYourSelf(self):
        rDebug("Killing myself")
        self.kill.emit()

    # \brief Change compute period
    # @param per Period in ms
    @QtCore.Slot(int)
    def setPeriod(self, p):
        print("Period changed", p)
        self.Period = p
        self.timer.start(self.Period)
