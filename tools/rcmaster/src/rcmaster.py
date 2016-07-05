#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright (C) 2016 by YOUR NAME HERE
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

# \mainpage RoboComp::rcmaster
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
# Just: "${PATH_TO_BINARY}/rcmaster --Ice.Config=${PATH_TO_CONFIG_FILE}"
#
# \subsection running_ssec Once running
#
#
#

import sys, traceback, Ice, IceStorm, subprocess, threading, time, Queue, os, copy

# Ctrl+c handling
import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)

from PySide import *

from specificworker import *

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP) < 1:
    print 'ROBOCOMP environment variable not set! Exiting.'
    sys.exit()

preStr = "-I" + ROBOCOMP + "/interfaces/ -I/opt/robocomp/interfaces/ --all " + ROBOCOMP + "/interfaces/"
Ice.loadSlice(preStr + "CommonBehavior.ice")
import RoboCompCommonBehavior

Ice.loadSlice(preStr + "RCMaster.ice")
import RoboCompRCMaster


class CommonBehaviorI(RoboCompCommonBehavior.CommonBehavior):
    def __init__(self, _handler, _communicator):
        self.handler = _handler
        self.communicator = _communicator

    def getFreq(self, current=None):
        self.handler.getFreq()

    def setFreq(self, freq, current=None):
        self.handler.setFreq()

    def timeAwake(self, current=None):
        try:
            return self.handler.timeAwake()
        except:
            print 'Problem getting timeAwake'

    def killYourSelf(self, current=None):
        self.handler.killYourSelf()

    def getAttrList(self, current=None):
        try:
            return self.handler.getAttrList(self.communicator)
        except:
            print 'Problem getting getAttrList'
            traceback.print_exc()
            status = 1
            return


if __name__ == '__main__':
    app = QtCore.QCoreApplication(sys.argv)
    params = copy.deepcopy(sys.argv)
    if len(params) > 1:
        if not params[1].startswith('--Ice.Config='):
            params[1] = '--Ice.Config=' + params[1]
    elif len(params) == 1:
        params.append('--Ice.Config=config')
    ic = Ice.initialize(params)
    status = 0
    mprx = {}

    try:

        # Remote object connection for DifferentialRobot
        try:
            mprx["databasePath"] = ic.getProperties().getProperty('rcmster.dbPath')
            mprx["cachettyl"] = ic.getProperties().getProperty('rcmaster.cachettyl')
            mprx["componentsToStart"] = ic.getProperties().getProperty('rcmaster.componentsToStart').split(',')

            if '' in mprx.values(): # @TODO improve
                print mprx.values()
                raise Ice.UserException("Cannot get all properties.")
        except Ice.Exception, e:
            print e
            print 'Cannot get all properties.'
            status = 1

    except:
        traceback.print_exc()
        status = 1

    if status == 0:
        worker = SpecificWorker(mprx)

        adapter = ic.createObjectAdapter('rcmaster')
        adapter.add(rcmasterI(worker), ic.stringToIdentity('rcmaster'))
        adapter.activate()
        masteruri = adapter.getPublishedEndpoints()[0].toString().split(" ")
        
        #write to config file
        try:
            f = open(os.path.join(os.path.expanduser('~'), ".config/RoboComp/rcmaster.config"),'r+')
            configs = f.read().splitlines()
            if len(configs) == 0:configs = ['']
            f.close()
        except :
            configs = ['']
        f = open(os.path.join(os.path.expanduser('~'), ".config/RoboComp/rcmaster.config"), 'w')
        configs[0] = str(masteruri[2] + ':' + masteruri[4])
        f.write("\n".join(configs));f.close()
        print "starting rcmaster on ",masteruri[2],"in port ",masteruri[4]
        
        #       adapter.add(CommonBehaviorI(<LOWER>I, ic), ic.stringToIdentity('commonbehavior'))

        app.exec_()

    if ic:
        try:
            ic.destroy()
        except:
            traceback.print_exc()
            status = 1
