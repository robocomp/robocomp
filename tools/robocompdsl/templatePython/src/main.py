#!/usr/bin/env python3
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

from templatePython.functions import main_py as main
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import get_name_number, IDSLPool, communication_is_ice
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)


pool = IDSLPool(theIDSLs, includeDirectories)
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

[[[cog
cog.out('# \\mainpage RoboComp::'+component.name)
]]]
[[[end]]]
#
# \section intro_sec Introduction
#
# Some information about the component...
#
# \section interface_sec Interface
#
# Descroption of the interface provided...
#
# \section install_sec Installation
#
# \subsection install1_ssec Software depencences
# Software dependences....
#
# \subsection install2_ssec Compile and install
# How to compile/install the component...
#
# \section guide_sec User guide
#
# \subsection config_ssec Configuration file
#
# <p>
# The configuration file...
# </p>
#
# \subsection execution_ssec Execution
#
[[[cog
cog.out('# Just: "${PATH_TO_BINARY}/'+component.name+' --Ice.Config=${PATH_TO_CONFIG_FILE}"')
]]]
[[[end]]]
#
# \subsection running_ssec Once running
#
#
#

import sys, traceback, IceStorm, time, os, copy

# Ctrl+c handling
import signal

from PySide2 import QtCore
[[[cog
    if component.gui is not None:
        cog.outl('from PySide2 import QtWidgets')
]]]
[[[end]]]

from specificworker import *


class CommonBehaviorI(RoboCompCommonBehavior.CommonBehavior):
    def __init__(self, _handler):
        self.handler = _handler
    def getFreq(self, current = None):
        self.handler.getFreq()
    def setFreq(self, freq, current = None):
        self.handler.setFreq()
    def timeAwake(self, current = None):
        try:
            return self.handler.timeAwake()
        except:
            print('Problem getting timeAwake')
    def killYourSelf(self, current = None):
        self.handler.killYourSelf()
    def getAttrList(self, current = None):
        try:
            return self.handler.getAttrList()
        except:
            print('Problem getting getAttrList')
            traceback.print_exc()
            status = 1
            return

#SIGNALS handler
def sigint_handler(*args):
    QtCore.QCoreApplication.quit()
    
if __name__ == '__main__':
    [[[cog
        if component.gui is not None:
            cog.outl('app = QtWidgets.QApplication(sys.argv)')
        else:
            cog.outl('app = QtCore.QCoreApplication(sys.argv)')
    ]]]
    [[[end]]]
    params = copy.deepcopy(sys.argv)
    if len(params) > 1:
        if not params[1].startswith('--Ice.Config='):
            params[1] = '--Ice.Config=' + params[1]
    elif len(params) == 1:
        params.append('--Ice.Config=etc/config')
    ic = Ice.initialize(params)
    status = 0
    mprx = {}
    parameters = {}
    for i in ic.getProperties():
        parameters[str(i)] = str(ic.getProperties().getProperty(i))
    [[[cog
    cog.out(main.storm_topic_manager_creation(component))

    cog.out(main.require_proxy_creation(component))

    cog.out(main.publish_proxy_creation(component))
    ]]]
    [[[end]]]

    if status == 0:
        worker = SpecificWorker(mprx)
        worker.setParams(parameters)
    else:
        print("Error getting required connections, check config file")
        sys.exit(-1)
    [[[cog
    cog.out(main.implements_adapters_creation(component))
    cog.out(main.subscribes_adapters_creation(component))
    ]]]
    [[[end]]]
    [[[cog
    cog.out(main.ros_service_and_subscribe_creation(component, pool))
    ]]]
    [[[end]]]

    signal.signal(signal.SIGINT, sigint_handler)
    app.exec_()

    if ic:
        try:
            ic.destroy()
        except:
            traceback.print_exc()
            status = 1
