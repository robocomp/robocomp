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
#    but WITHOUT ANY WARRANTY without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import Ice, sys, math, traceback

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *

class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.t = 0.
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompTrajectoryRobot2D'].TrajectoryRobot2DPrx.checkedCast(self.prx)

		self.positionXLabel = QLabel("X", self)
		self.positionXLabel.show()
		self.positionXLabel.move(5,8)
		self.positionX = QSpinBox(self)
		self.positionX.setMaximum(10000)
		self.positionX.setMinimum(-10000)
		self.positionX.show()
		self.positionX.setValue(0)
		self.positionX.setSingleStep(25)
		self.positionX.move(50,8)

		self.positionZLabel = QLabel("Z", self)
		self.positionZLabel.show()
		self.positionZLabel.move(5,33)
		self.positionZ = QSpinBox(self)
		self.positionZ.setMaximum(10000)
		self.positionZ.setMinimum(-10000)
		self.positionZ.show()
		self.positionZ.setValue(0)
		self.positionZ.setSingleStep(25)
		self.positionZ.move(50,33)
		
		self.steerLabel = QLabel('Angle', self)
		self.steerLabel.show()
		self.steerLabel.move(5,58)
		self.steer = QDoubleSpinBox(self)
		self.steer.setMaximum(3.14*10)
		self.steer.setMinimum(-3.14*10)
		self.steer.setSingleStep(0.05)
		self.steer.setValue(0)
		self.steer.move(50,58)
		self.steer.show()

		self.resetButton = QPushButton("go", self)
		self.resetButton.move(5,90)
		self.resetButton.show()
		self.connect(self.resetButton, SIGNAL('clicked()'), self.go)
		self.show()
		
	def job(self):
		pass


	def go(self):
		print 'a1'
		tp = self.mods['RoboCompTrajectoryRobot2D'].TargetPose()
		tp.x = self.positionX.value()
		tp.z = self.positionZ.value()
		tp.y = 0
		tp.rx = 0
		tp.ry = self.steer.value()
		tp.rz = 0
		tp.onlyRot = True
		self.proxy.go(tp)
		print 'a2'

