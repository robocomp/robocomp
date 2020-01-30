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

import RoboCompLaser
global RoboCompLaser


replay_plugin_identifier = 'laser_hal'


def getReplayClass():
	return LaserI()
def getRecordClass(proxy):
	return LaserRecorder(proxy)
def getGraphicalUserInterface():
	return LaserGUI()


class LaserGUI(QWidget):
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
			self.painter.setPen(QColor(0, 0, 255, 150))
			for p in self.measure[0]:
				x =  float(p.dist) * math.sin(p.angle)
				y = - float(p.dist) * math.cos(p.angle)
				self.painter.drawRect(0.02*x+self.width()/2, 0.02*y+self.height()/2, 1, 1)
			self.painter = None

class LaserI(RoboCompLaser.Laser):
	def __init__(self):
		self.measure = None
		self.configuration = None
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def setMeasure(self, measure):
		self.measure = measure
	def getMeasure(self):
		return self.measure
	def getLaserData(self, current = None):
		return self.measure[0]
	def getLaserAndBStateData(self, current = None):
		return self.measure
	def getLaserConfData(self, current = None):
		return self.configuration

class LaserRecorder:
	def __init__(self, proxy):
		global RoboCompLaser
		self.proxy = RoboCompLaser.LaserPrx.checkedCast(proxy)
	def getConfiguration(self):
		return self.proxy.getLaserConfData()
	def getMeasure(self):
		self.measure = self.proxy.getLaserAndBStateData()
		return self.measure
	def measure(self):
		return self.measure




