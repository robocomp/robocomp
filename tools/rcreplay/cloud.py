# -*- coding: utf-8 -*-

#    Copyright (C) 2010 by RoboLab - University of Extremadura
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

import Ice, threading
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
import math
import os




import RoboCompCloudPrimitives
global RoboCompCloudPrimitives


replay_plugin_identifier = 'cloudprimitives'


def getReplayClass():
	return CloudPrimitivesI()
def getRecordClass(proxy):
	return CloudPrimitivesRecorder(proxy)
def getGraphicalUserInterface():
	return CloudPrimitivesGUI()


class CloudPrimitivesGUI(QWidget):
	def __init__(self, parent=None):
		QWidget.__init__(self,parent)
		self.show()
		self.measure = None
		self.configuration = None
	def getSize(self):
		return QSize(500, 500)
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def setMeasure(self, measure):
		self.measure = measure
	def paintEvent(self, event):
		if self.measure:
			self.painter = QPainter(self)
		self.painter = None

class CloudPrimitivesI(RoboCompCloudPrimitives.CloudPrimitives):
	def __init__(self):
		self.measure = None
		self.configuration = None
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def setMeasure(self, measure):
		self.measure = measure
	def getMeasure(self):
		return self.measure
	def getPatches(self, current = None):
		return self.measure[1] # Patches
	def getData(self, current = None):
		return self.measure[0] # Data


class CloudPrimitivesRecorder:
	def __init__(self, proxy):
		global RoboCompCloudPrimitives
		self.proxy = RoboCompCloudPrimitives.CloudPrimitivesPrx.checkedCast(proxy)
		self.numMeasure = 0
	def getConfiguration(self):
		return True
	def getMeasure(self):
		print 'Reading measure'
		self.numMeasure = self.numMeasure + 1
		self.measure = [self.proxy.getData(), self.proxy.getPatches()]
		#print ('Measures read', self.numMeasure)
		#os.system('mplayer /home/robolab/beep.wav')
		return self.measure
	def measure(self):
		return self.measure


