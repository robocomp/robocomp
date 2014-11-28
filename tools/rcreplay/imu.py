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
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *
import math



import RoboCompIMU
global RoboCompIMU


replay_plugin_identifier = 'imu_hal'


def getReplayClass():
	return IMUI()
def getRecordClass(proxy):
	return IMURecorder(proxy)
def getGraphicalUserInterface():
	return IMUGUI()


class IMUGUI(QWidget):
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

class IMUI(RoboCompIMU.IMU):
	def __init__(self):
		self.measure = None
		self.configuration = None
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def setMeasure(self, measure):
		self.measure = measure
	def getMeasure(self):
		return self.measure
	def getDataImu(self, current = None):
		return self.measure
	def getAcceleration(self, current = None):
		global RoboCompIMU
		acc = RoboCompIMU.Acceleration()
		acc.XAcc = self.measure.XAcc
		acc.YAcc = self.measure.YAcc
		acc.ZAcc = self.measure.ZAcc
		print acc.XAcc, acc.YAcc, acc.ZAcc
		return acc
	def getVelangular(self, current = None):
		return True
	def getMagneticfields(self, current = None):
		return True
	def getOrientation(self, current = None):
		return True

class IMURecorder:
	def __init__(self, proxy):
		global RoboCompIMU
		self.proxy = RoboCompIMU.IMUPrx.checkedCast(proxy)
	def getConfiguration(self):
		return True
	def getMeasure(self):
		self.measure = self.proxy.getDataImu()
		return self.measure
	def measure(self):
		return self.measure


