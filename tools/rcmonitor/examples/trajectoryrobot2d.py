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
from random import uniform

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


		self.positionRangeXLabel = QLabel("Range X", self)
		self.positionRangeXLabel.show()
		self.positionRangeXLabel.move(205,8)
		self.positionRangeXmin = QSpinBox(self)
		self.positionRangeXmin.setMaximum(10000)
		self.positionRangeXmin.setMinimum(-10000)
		self.positionRangeXmin.show()
		self.positionRangeXmin.setValue(0)
		self.positionRangeXmin.setSingleStep(25)
		self.positionRangeXmin.move(255,8)
		self.positionRangeXmax = QSpinBox(self)
		self.positionRangeXmax.setMaximum(10000)
		self.positionRangeXmax.setMinimum(-10000)
		self.positionRangeXmax.show()
		self.positionRangeXmax.setValue(0)
		self.positionRangeXmax.setSingleStep(25)
		self.positionRangeXmax.move(335,8)


		self.positionRangeZLabel = QLabel("Range Z", self)
		self.positionRangeZLabel.show()
		self.positionRangeZLabel.move(205,33)
		self.positionRangeZmin = QSpinBox(self)
		self.positionRangeZmin.setMaximum(10000)
		self.positionRangeZmin.setMinimum(-10000)
		self.positionRangeZmin.show()
		self.positionRangeZmin.setValue(0)
		self.positionRangeZmin.setSingleStep(25)
		self.positionRangeZmin.move(255,33)
		self.positionRangeZmax = QSpinBox(self)
		self.positionRangeZmax.setMaximum(10000)
		self.positionRangeZmax.setMinimum(-10000)
		self.positionRangeZmax.show()
		self.positionRangeZmax.setValue(0)
		self.positionRangeZmax.setSingleStep(25)
		self.positionRangeZmax.move(335,33)

		self.steerRangeLabel = QLabel('R.Angle', self)
		self.steerRangeLabel.show()
		self.steerRangeLabel.move(205,58)
		self.steerRangemin = QDoubleSpinBox(self)
		self.steerRangemin.setMaximum(3.14*10)
		self.steerRangemin.setMinimum(-3.14*10)
		self.steerRangemin.setSingleStep(0.05)
		self.steerRangemin.setValue(0)
		self.steerRangemin.move(255,58)
		self.steerRangemin.show()
		
		self.steerRangemax = QDoubleSpinBox(self)
		self.steerRangemax.setMaximum(3.14*10)
		self.steerRangemax.setMinimum(-3.14*10)
		self.steerRangemax.setSingleStep(0.05)
		self.steerRangemax.setValue(0)
		self.steerRangemax.move(335,58)
		self.steerRangemax.show()
		w
		self.numTargetsLabel = QLabel('Targets', self)
		self.numTargetsLabel.show()
		self.numTargetsLabel.move(205,80)
		self.numTargets = QSpinBox(self)
		self.numTargets.setMaximum(10000)
		self.numTargets.setMinimum(-10000)
		self.numTargets.show()
		self.numTargets.setValue(0)
		self.numTargets.setSingleStep(25)
		self.numTargets.move(255,80)

		self.resetButton = QPushButton("go", self)
		self.resetButton.move(5,90)
		self.resetButton.show()
		self.connect(self.resetButton, SIGNAL('clicked()'), self.go)
		self.show()

		self.resetButton = QPushButton("goAll", self)
		self.resetButton.move(355,90)
		self.resetButton.show()
		self.connect(self.resetButton, SIGNAL('clicked()'), self.goAll)
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

	def goAll(self):
		print 'a1'
		tp = self.mods['RoboCompTrajectoryRobot2D'].TargetPose()

		minRangeX = self.positionRangeXmin.value()
		maxRangeX = self.positionRangeXmax.value()
		minRangeZ = self.positionRangeZmin.value()
		maxRangeZ = self.positionRangeZmax.value()
		minRangeSteer = self.steerRangemin.value()
		maxRangeSteer = self.steerRangemax.value()
		numTargets = self.numTargets.value()

		while numTargets > 0:			
			tp.x = uniform(minRangeX,maxRangeX)
			tp.z = uniform(minRangeZ,maxRangeZ)
			tp.y = 0
			tp.rx = 0
			tp.ry = uniform(minRangeSteer,maxRangeSteer)
			tp.rz = 0
			self.proxy.go(tp)
			while True:
				state = self.proxy.getState()
				state = state.state
				if state == "IDLE":
					print "in target number: "+str(self.numTargets.value() - numTargets)
					numTargets -= 1
					break
