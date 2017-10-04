#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  -----------------------
#  -----  rcmanager  -----
#  -----------------------
#  An ICE component manager.
#
#    Copyright (C) 2009-2015 by RoboLab - University of Extremadura
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
#

#
# CODE BEGINS
#

from PyQt4 import QtCore

class RCManagerSignals(QtCore.QObject):

    sample = QtCore.pyqtSignal()
    modelIsReady = QtCore.pyqtSignal()
    viewerIsReady = QtCore.pyqtSignal()
    controllerIsReady = QtCore.pyqtSignal(str)
    saveModel = QtCore.pyqtSignal(str)
    openModel = QtCore.pyqtSignal(str, bool)
    closeModel = QtCore.pyqtSignal()
    addNode = QtCore.pyqtSignal(dict)
    startComponent = QtCore.pyqtSignal(str)
    stopComponent = QtCore.pyqtSignal(str)
    componentRunning = QtCore.pyqtSignal(str)
    componentStopped = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        QtCore.QObject.__init__(self)

CustomSignalCollection = RCManagerSignals()
