#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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

Ice.loadSlice("-I ./src/ --all ./src/CommonBehavior.ice")
import RoboCompCommonBehavior

Ice.loadSlice("-I ./src/ --all ./src/AprilBasedLocalization.ice")
import RoboCompAprilBasedLocalization
Ice.loadSlice("-I ./src/ --all ./src/AprilTags.ice")
import RoboCompAprilTags
Ice.loadSlice("-I ./src/ --all ./src/CameraSimple.ice")
import RoboCompCameraSimple
Ice.loadSlice("-I ./src/ --all ./src/GenericBase.ice")
import RoboCompGenericBase
Ice.loadSlice("-I ./src/ --all ./src/HandDetection.ice")
import RoboCompHandDetection
Ice.loadSlice("-I ./src/ --all ./src/JointMotor.ice")
import RoboCompJointMotor
Ice.loadSlice("-I ./src/ --all ./src/RGBD.ice")
import RoboCompRGBD

class tagsList(list):
    def __init__(self, iterable=list()):
        super(tagsList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompAprilTags.tag)
        super(tagsList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompAprilTags.tag)
        super(tagsList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompAprilTags.tag)
        super(tagsList, self).insert(index, item)

setattr(RoboCompAprilTags, "tagsList", tagsList)

class ImgType(list):
    def __init__(self, iterable=list()):
        super(ImgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(ImgType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(ImgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(ImgType, self).insert(index, item)

setattr(RoboCompCameraSimple, "ImgType", ImgType)

class ImgType(list):
    def __init__(self, iterable=list()):
        super(ImgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(ImgType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(ImgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(ImgType, self).insert(index, item)

setattr(RoboCompHandDetection, "ImgType", ImgType)

class KeyPoint(list):
    def __init__(self, iterable=list()):
        super(KeyPoint, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, int)
        super(KeyPoint, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, int)
        super(KeyPoint, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, int)
        super(KeyPoint, self).insert(index, item)

setattr(RoboCompHandDetection, "KeyPoint", KeyPoint)

class TCoordSequence(list):
    def __init__(self, iterable=list()):
        super(TCoordSequence, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHandDetection.KeyPoint)
        super(TCoordSequence, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHandDetection.KeyPoint)
        super(TCoordSequence, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHandDetection.KeyPoint)
        super(TCoordSequence, self).insert(index, item)

setattr(RoboCompHandDetection, "TCoordSequence", TCoordSequence)

class TContour(list):
    def __init__(self, iterable=list()):
        super(TContour, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHandDetection.TCoordSequence)
        super(TContour, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHandDetection.TCoordSequence)
        super(TContour, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHandDetection.TCoordSequence)
        super(TContour, self).insert(index, item)

setattr(RoboCompHandDetection, "TContour", TContour)

class Hands(list):
    def __init__(self, iterable=list()):
        super(Hands, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHandDetection.Hand)
        super(Hands, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHandDetection.Hand)
        super(Hands, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHandDetection.Hand)
        super(Hands, self).insert(index, item)

setattr(RoboCompHandDetection, "Hands", Hands)

class MotorParamsList(list):
    def __init__(self, iterable=list()):
        super(MotorParamsList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompJointMotor.MotorParams)
        super(MotorParamsList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompJointMotor.MotorParams)
        super(MotorParamsList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompJointMotor.MotorParams)
        super(MotorParamsList, self).insert(index, item)

setattr(RoboCompJointMotor, "MotorParamsList", MotorParamsList)

class MotorGoalPositionList(list):
    def __init__(self, iterable=list()):
        super(MotorGoalPositionList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompJointMotor.MotorGoalPosition)
        super(MotorGoalPositionList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompJointMotor.MotorGoalPosition)
        super(MotorGoalPositionList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompJointMotor.MotorGoalPosition)
        super(MotorGoalPositionList, self).insert(index, item)

setattr(RoboCompJointMotor, "MotorGoalPositionList", MotorGoalPositionList)

class MotorGoalVelocityList(list):
    def __init__(self, iterable=list()):
        super(MotorGoalVelocityList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompJointMotor.MotorGoalVelocity)
        super(MotorGoalVelocityList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompJointMotor.MotorGoalVelocity)
        super(MotorGoalVelocityList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompJointMotor.MotorGoalVelocity)
        super(MotorGoalVelocityList, self).insert(index, item)

setattr(RoboCompJointMotor, "MotorGoalVelocityList", MotorGoalVelocityList)

class MotorList(list):
    def __init__(self, iterable=list()):
        super(MotorList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, str)
        super(MotorList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, str)
        super(MotorList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, str)
        super(MotorList, self).insert(index, item)

setattr(RoboCompJointMotor, "MotorList", MotorList)

class imgType(list):
    def __init__(self, iterable=list()):
        super(imgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(imgType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(imgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(imgType, self).insert(index, item)

setattr(RoboCompRGBD, "imgType", imgType)

class depthType(list):
    def __init__(self, iterable=list()):
        super(depthType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(depthType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, float)
        super(depthType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(depthType, self).insert(index, item)

setattr(RoboCompRGBD, "depthType", depthType)

class DepthSeq(list):
    def __init__(self, iterable=list()):
        super(DepthSeq, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(DepthSeq, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, float)
        super(DepthSeq, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(DepthSeq, self).insert(index, item)

setattr(RoboCompRGBD, "DepthSeq", DepthSeq)

class ColorSeq(list):
    def __init__(self, iterable=list()):
        super(ColorSeq, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompRGBD.ColorRGB)
        super(ColorSeq, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompRGBD.ColorRGB)
        super(ColorSeq, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompRGBD.ColorRGB)
        super(ColorSeq, self).insert(index, item)

setattr(RoboCompRGBD, "ColorSeq", ColorSeq)

class PointSeq(list):
    def __init__(self, iterable=list()):
        super(PointSeq, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompRGBD.PointXYZ)
        super(PointSeq, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompRGBD.PointXYZ)
        super(PointSeq, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompRGBD.PointXYZ)
        super(PointSeq, self).insert(index, item)

setattr(RoboCompRGBD, "PointSeq", PointSeq)


import handdetectionI
import apriltagsI


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
