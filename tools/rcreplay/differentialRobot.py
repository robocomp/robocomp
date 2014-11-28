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

import Ice, threading, math
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *




import RoboCompDifferentialRobot
global RoboCompDifferentialRobot


replay_plugin_identifier = 'differentialrobot_hal'


def getReplayClass():
	return DifferentialRobotI()
def getRecordClass(proxy):
	return DifferentialRobotRecorder(proxy)
def getGraphicalUserInterface():
	return DifferentialRobotGUI()


class DifferentialRobotGUI(QWidget):
	def __init__(self, parent=None):
		QWidget.__init__(self,parent)
		self.show()
		self.measure = None
		self.configuration = None
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def getSize(self):
		return QSize(500, 500)
	def setMeasure(self, measure):
		self.measure = measure
	def paintEvent(self, event):
		if self.measure:
			self.painter = QPainter(self)
			self.painter.setRenderHint(QPainter.Antialiasing, True)
			self.painter.setPen(QColor(0, 0, 255, 150))
			self.painter.drawLine(self.width()/2, 0, self.width()/2, self.height())
			self.painter.drawLine(0, self.height()/2, self.width(), self.height()/2)
			x =  0.1*float(self.measure.x)+float(self.width())/2.
			y = -0.1*float(self.measure.z)+float(self.height())/2.
			self.painter.setPen(QColor(0, 0, 0, 150))
			self.painter.setBrush(Qt.lightGray)
			self.painter.drawEllipse(x-14., y-14., 28., 28.)
			self.painter.setBrush(Qt.darkRed)
			p = 0.35
			xc = x + 12.*math.sin(float(self.measure.alpha)+p)
			yc = y - 12.*math.cos(float(self.measure.alpha)+p)
			self.painter.drawEllipse(xc-4., yc-4., 8., 8.)
			p = -0.35
			xc = x + 12.*math.sin(float(self.measure.alpha)+p)
			yc = y - 12.*math.cos(float(self.measure.alpha)+p)
			self.painter.drawEllipse(xc-4., yc-4., 8., 8.)
			self.painter = None


class DifferentialRobotI(RoboCompDifferentialRobot.DifferentialRobot):
	def __init__(self):
		self.state = RoboCompDifferentialRobot.TBaseState()
		self.configuration = None
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def setMeasure(self, measure):
		self.state = measure
	def getMeasure(self):
		return self.state
	#void getBaseState(out TBaseState state);
	def getBaseState(self, current = None):
		return self.state
	#void getBasePose(out int x, out int z, out float alfa);
	def getBasePose(self, current = None):
		return self.state.x, self.state.z, self.state.alpha
	#void setSpeedBase(float adv, float rot);
	def setSpeedBase(self, adv, rot, current = None):
		pass
	#void stopBase();
	def stopBase(self, current = None):
		pass
	#void resetOdometer();
	def resetOdometer(self, current = None):
		pass
	#void setOdometer(TBaseState state);
	def setOdometer(self, state, current = None):
		pass
	#void setOdometerPose(int x, int z, float alfa);
	def setOdometerPose(self, x, z, a, current = None):
		pass


class DifferentialRobotRecorder:
	def __init__(self, proxy):
		self.proxy = RoboCompDifferentialRobot.DifferentialRobotPrx.checkedCast(proxy)
	def getConfiguration(self):
		return 'Nada por ahora'
	def getMeasure(self):
		self.state = self.proxy.getBaseState()
		return self.state
	def measure(self):
		return self.state

