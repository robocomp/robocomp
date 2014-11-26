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



import RoboCompRGBD
global RoboCompRGBD


replay_plugin_identifier = 'rgbd_hal'


def getReplayClass():
	return RGBDI()
def getRecordClass(proxy):
	return RGBDRecorder(proxy)
def getGraphicalUserInterface():
	return RGBDGUI()


class RGBDGUI(QWidget):
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
			self.painter.setRenderHint(QPainter.Antialiasing, True)
			image = QImage(self.measure[0], 640, 480, QImage.Format_RGB888)
			self.painter.drawImage(QPointF(0, 0), image)
		self.painter = None

class RGBDI(RoboCompRGBD.RGBD):
	def __init__(self):
		self.measure = None
		self.configuration = None
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def setMeasure(self, measure):
		self.measure = measure
	def getMeasure(self):
		return self.measure
	def getData(self, current = None):
		return self.measure
	def getRGBDParams(self, current = None):
		return self.configuration

class RGBDRecorder:
	def __init__(self, proxy):
		global RoboCompRGBD
		self.proxy = RoboCompRGBD.RGBDPrx.checkedCast(proxy)
	def getConfiguration(self):
		return self.proxy.getRGBDParams()
	def getMeasure(self):
		self.measure = self.proxy.getData()
		return self.measure
	def measure(self):
		return self.measure














































