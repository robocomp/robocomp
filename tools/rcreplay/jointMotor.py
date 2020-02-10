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
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


import RoboCompJointMotor
global RoboCompJointMotor


replay_plugin_identifier = 'jointmotor_hal'


def getReplayClass():
	return JointMotorI()
def getRecordClass(proxy):
	return JointMotorRecorder(proxy)
def getGraphicalUserInterface():
	return JointMotorGUI()


class JointMotorGUI(QWidget):
	def __init__(self, parent=None):
		QWidget.__init__(self,parent)
		self.show()
		self.measure = None
		self.backMeasures = dict()
		self.configuration = None
	def getSize(self):
		return QSize(300, 300)
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def setMeasure(self, measure):
		self.measure = measure
		for motor in self.measure:
			if not motor in self.backMeasures:
				self.backMeasures[motor] = [QLabel(QString(motor), self), QLabel(QString('000000000000'), self)]
				self.backMeasures[motor][0].show()
				self.backMeasures[motor][1].show()
				self.backMeasures[motor][0].move(0, self.backMeasures[motor][0].height()*(len(self.backMeasures)-1))
				self.backMeasures[motor][1].move(100, self.backMeasures[motor][0].height()*(len(self.backMeasures)-1))
			self.backMeasures[motor][1].setText(QString.number(self.measure[motor].pos))
		if len(self.backMeasures)>0:
			key = self.backMeasures.keys()[0]
			self.resize(100+self.backMeasures[key][1].width(), self.backMeasures[key][0].height()*(len(self.backMeasures)))



class JointMotorI(RoboCompJointMotor.JointMotor):
	def __init__(self):
		self.state = None
		self.configuration = None
	def setConfiguration(self, configuration):
		print replay_plugin_identifier, 'setting configuration', configuration
		self.configuration = configuration
	def setMeasure(self, measure):
		self.state = measure
	def getMeasure(self):
		return self.state
	# void setPosition(MotorGoalPosition goal)
	def setPosition(self, goal, current = None):
		pass
	# void setVelocity(MotorGoalVelocity goal)
	def setVelocity(self, goal, current = None):
		pass
	# void setSyncPosition(MotorGoalPositionList listGoals)
	def setSyncPosition(self, listGoals, current = None):
		pass
	# MotorParams getMotorParams(string motor)
	def getMotorParams(self, motor, current = None):
		return self.configuration[1][motor]
	# MotorState getMotorState(string motor)
	def getMotorState(self, motor, current = None):
		return self.state[motor]
	# MotorStateMap getMotorStateMap(MotorList mList)
	def getMotorStateMap(self, mList, current = None):
		ret = dict()
		for identifier in mList:
			ret[identifier] = self.state[identifier]
		return ret
	# MotorStateMap getAllMotorState()
	def getAllMotorState(self, current = None):
		return self.state
	# MotorParamsList getAllMotorParams()
	def getAllMotorParams(self, current = None):
		return self.configuration[1]
	# BusParams getBusParams()
	def getBusParams(self, current = None):
		return self.configuration[0]


class JointMotorRecorder:
	def __init__(self, proxy):
		self.proxy = RoboCompJointMotor.JointMotorPrx.checkedCast(proxy)
	def getConfiguration(self):
		return [ self.proxy.getBusParams(), self.proxy.getAllMotorParams() ]
	def getMeasure(self):
		self.state = self.proxy.getAllMotorState()
		return self.state
	def measure(self):
		return self.state

