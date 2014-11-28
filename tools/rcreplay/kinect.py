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
import os




import RoboCompKinect
global RoboCompKinect


replay_plugin_identifier = 'kinect_hal'


def getReplayClass():
	return KinectI()
def getRecordClass(proxy):
	return KinectRecorder(proxy)
def getGraphicalUserInterface():
	return KinectGUI()


class KinectGUI(QWidget):
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
			#self.painter.setRenderHint(QPainter.Antialiasing, True)
			#image = QImage(self.measure[0], 640, 480, QImage.Format_RGB888)
			#self.painter.drawImage(QPointF(0, 0), image)
		self.painter = None

class KinectI(RoboCompKinect.Kinect):
	def __init__(self):
		self.measure = None
		self.configuration = None
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def setMeasure(self, measure):
		self.measure = measure
	def getMeasure(self):
		return self.measure
	def getDataRGBZinRGB(self, current = None):
		return self.measure[0]
	def getDataRGBZinIR(self, current = None):
		return self.measure[0]
	def getDataIR(self, current = None):
		return
	def setTilt (self,angle,current = None):
		return
	def getTilt (self, current = None):
		return
	def setLed (self,led,current = None):
		return
	def getLed (self, current = None):
		return
	def getRGB(self, current = None):
		return self.measure[0][0]
	def getXYZ(self, current = None):
		return self.measure[1]

class KinectRecorder:
	def __init__(self, proxy):
		global RoboCompKinect
		self.proxy = RoboCompKinect.KinectPrx.checkedCast(proxy)
		self.numMeasure = 0
	def getConfiguration(self):
		return True
	def getMeasure(self):
		print 'Reading measure'
		self.numMeasure = self.numMeasure + 1
		self.measure = [self.proxy.getDataRGBZinRGB(), self.proxy.getXYZ()]
		print 'Measures read', self.numMeasure
		os.system('mplayer /home/robolab/beep.wav')
		return self.measure
	def measure(self):
		return self.measure

