#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2020 by YOUR NAME HERE
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

ice_AprilBasedLocalization = False
for p in icePaths:
    if os.path.isfile(p+'/AprilBasedLocalization.ice'):
        preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
        wholeStr = preStr+"AprilBasedLocalization.ice"
        Ice.loadSlice(wholeStr)
        ice_AprilBasedLocalization = True
        break
if not ice_AprilBasedLocalization:
    print('Couln\'t load AprilBasedLocalization')
    sys.exit(-1)
from RoboCompAprilBasedLocalization import *

ice_AprilTags = False
for p in icePaths:
    if os.path.isfile(p+'/AprilTags.ice'):
        preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
        wholeStr = preStr+"AprilTags.ice"
        Ice.loadSlice(wholeStr)
        ice_AprilTags = True
        break
if not ice_AprilTags:
    print('Couln\'t load AprilTags')
    sys.exit(-1)
from RoboCompAprilTags import *

ice_CameraSimple = False
for p in icePaths:
    if os.path.isfile(p+'/CameraSimple.ice'):
        preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
        wholeStr = preStr+"CameraSimple.ice"
        Ice.loadSlice(wholeStr)
        ice_CameraSimple = True
        break
if not ice_CameraSimple:
    print('Couln\'t load CameraSimple')
    sys.exit(-1)
from RoboCompCameraSimple import *

ice_GenericBase = False
for p in icePaths:
    if os.path.isfile(p+'/GenericBase.ice'):
        preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
        wholeStr = preStr+"GenericBase.ice"
        Ice.loadSlice(wholeStr)
        ice_GenericBase = True
        break
if not ice_GenericBase:
    print('Couln\'t load GenericBase')
    sys.exit(-1)
from RoboCompGenericBase import *

ice_HandDetection = False
for p in icePaths:
    if os.path.isfile(p+'/HandDetection.ice'):
        preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
        wholeStr = preStr+"HandDetection.ice"
        Ice.loadSlice(wholeStr)
        ice_HandDetection = True
        break
if not ice_HandDetection:
    print('Couln\'t load HandDetection')
    sys.exit(-1)
from RoboCompHandDetection import *

ice_JointMotor = False
for p in icePaths:
    if os.path.isfile(p+'/JointMotor.ice'):
        preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
        wholeStr = preStr+"JointMotor.ice"
        Ice.loadSlice(wholeStr)
        ice_JointMotor = True
        break
if not ice_JointMotor:
    print('Couln\'t load JointMotor')
    sys.exit(-1)
from RoboCompJointMotor import *

ice_RGBD = False
for p in icePaths:
    if os.path.isfile(p+'/RGBD.ice'):
        preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
        wholeStr = preStr+"RGBD.ice"
        Ice.loadSlice(wholeStr)
        ice_RGBD = True
        break
if not ice_RGBD:
    print('Couln\'t load RGBD')
    sys.exit(-1)
from RoboCompRGBD import *

from handdetectionI import *
from apriltagsI import *


try:
    from ui_mainUI import *
except:
    print("Can't import UI file. Did you run 'make'?")
    sys.exit(-1)



class GenericWorker(QtWidgets.QDialog):

    kill = QtCore.Signal()
    #Signals for State Machine
    t_initialize_to_compute = QtCore.Signal()
    t_compute_to_compute = QtCore.Signal()
    t_compute_to_finalize = QtCore.Signal()

    #-------------------------

    def __init__(self, mprx):
        super(GenericWorker, self).__init__()


        self.camerasimple_proxy = mprx["CameraSimpleProxy"]
        self.rgbd_proxy = mprx["RGBDProxy"]
        self.aprilbasedlocalization_proxy = mprx["AprilBasedLocalizationPub"]
        self.ui = Ui_guiDlg()
        self.ui.setupUi(self)
        self.show()

        self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
        self.Period = 30
        self.timer = QtCore.QTimer(self)

        #State Machine
        self.defaultMachine= QtCore.QStateMachine()
        self.compute_state = QtCore.QState(self.defaultMachine)
        self.initialize_state = QtCore.QState(self.defaultMachine)

        self.finalize_state = QtCore.QFinalState(self.defaultMachine)


        #------------------
        #Initialization State machine
        self.initialize_state.addTransition(self.t_initialize_to_compute, self.compute_state)
        self.compute_state.addTransition(self.t_compute_to_compute, self.compute_state)
        self.compute_state.addTransition(self.t_compute_to_finalize, self.finalize_state)


        self.compute_state.entered.connect(self.sm_compute)
        self.initialize_state.entered.connect(self.sm_initialize)
        self.finalize_state.entered.connect(self.sm_finalize)
        self.timer.timeout.connect(self.t_compute_to_compute)

        self.defaultMachine.setInitialState(self.initialize_state)

        #------------------

    #Slots funtion State Machine

    @QtCore.Slot()
    def sm_compute(self):
        print("Error: lack sm_compute in Specificworker")
        sys.exit(-1)

    @QtCore.Slot()
    def sm_initialize(self):
        print("Error: lack sm_initialize in Specificworker")
        sys.exit(-1)

    @QtCore.Slot()
    def sm_finalize(self):
        print("Error: lack sm_finalize in Specificworker")
        sys.exit(-1)

    #-------------------------
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
