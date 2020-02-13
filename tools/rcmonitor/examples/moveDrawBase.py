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

import Ice, sys, math, traceback

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompDifferentialRobot'].DifferentialRobotPrx.checkedCast(self.prx)
		self.speedLabel = QLabel("Speed", self)
		self.speedLabel.show()
		self.speedLabel.move(5,8)
		self.steerLabel = QLabel('Steer', self)
		self.steerLabel.show()
		self.steerLabel.move(5,33)
		self.speed = QSpinBox(self)
		self.speed.setMaximum(10000)
		self.speed.setMinimum(-100)
		self.speed.show()
		self.speed.setValue(0)
		self.speed.move(50,5)
		self.steer = QDoubleSpinBox(self)
		self.steer.setMaximum(3.14*10)
		self.steer.setMinimum(-3.14*10)
		self.steer.setValue(0)
		self.steer.move(50,30)
		self.steer.show()
		self.resetButton = QPushButton("reset", self)
		self.resetButton.move(5,55)
		self.resetButton.show()
		self.connect(self.resetButton, SIGNAL('clicked()'), self.reset)
		self.show()

		self.cmdSteer = 0.
		self.cmdSpeed = 0.

		self.posList = []

	def reset(self):
		self.proxy.resetOdometer()

	def job(self):
		sendCommand = False
		if self.cmdSteer != float(self.steer.value()):
			self.cmdSteer = float(self.steer.value())
			sendCommand = True
		if self.cmdSpeed != float(self.speed.value()):
			self.cmdSpeed = float(self.speed.value())
			sendCommand = True
#		sendCommand = True
		if sendCommand: self.proxy.setSpeedBase(float(self.speed.value()), float(self.steer.value()))
		self.bState = self.proxy.getBaseState()
		self.posList.append(self.bState)
		if len(self.posList) > 500:
			self.posList = self.posList[1:]

	def paintEvent(self, event=None):
		xOff = self.width()/2.
		yOff = self.height()/2.
		print (self.bState.x, self.bState.z, self.bState.alpha)

		xPos = 0
		yPos = 0
		div = 18.
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)
		# Draw grid
		for i in range(2+int(max(xOff,yOff)/30)):
			x = 30*i
			painter.drawLine(xOff+x, 0, xOff+x, self.height())
			painter.drawLine(xOff-x, 0, xOff-x, self.height())
			painter.drawLine(0, yOff+x, self.width(), yOff+x)
			painter.drawLine(0, yOff-x, self.width(), yOff-x)
		
		# Draw base
		painter.setPen(Qt.blue)
		painter.setBrush(Qt.blue)
		try:
			xPos = int( (self.bState.z/div)+xOff-9)
			yPos = int( (self.bState.x/div)+yOff-9)
			start = int(((-self.bState.alpha*180/math.pi)-180-20)*16)
			if type(xPos) == type(yPos) and type(xPos) == type(start) and type(xPos) == type(int()):
				try:
					painter.drawPie(xPos, yPos, 18, 18, start, 20*2*16)
				except:
					print ('BASE :-(')
					print (type(xPos-7))
					print (self.bState.z, self.bState.x, self.bState.alpha)
		except:
			pass

		# Draw Line
		for i in range(len(self.posList)):
			xPosLine = int( (self.posList[i].z/div)+xOff)
			yPosLine = int( (self.posList[i].x/div)+yOff)
			painter.drawPoint(xPosLine, yPosLine)

		painter.setPen(Qt.red)
		painter.setBrush(Qt.red)
		try:
			xPos = int( (self.bState.correctedZ/div)+xOff-9)
			yPos = int( (self.bState.correctedX/div)+yOff-9)
			start = int(((-self.bState.correctedAlpha*180/math.pi)-180-20)*16)
			if type(xPos) == type(yPos) and type(xPos) == type(start) and type(xPos) == type(int()):
				try:
					painter.drawPie(xPos, yPos, 18, 18, start, 20*2*16)
				except:
					print ('BASE :-(')
					print (type(xPos-7))
					print (self.bState.correctedZ, self.bState.ocrrectedX, self.bState.correctedAlpha)
		except:
			pass

		for i in range(len(self.posList)):
			xPosLine = int( (self.posList[i].correctedZ/div)+xOff)
			yPosLine = int( (self.posList[i].correctedX/div)+yOff)
			painter.drawPoint(xPosLine, yPosLine)

		painter.end()
		painter = None
