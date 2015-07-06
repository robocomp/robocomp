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


		self.a()
		#self.b()
		sys.exit(1)

	def job(self):
		pass


	def a(self):
		print 'a1'
		tp = self.mods['RoboCompTrajectoryRobot2D'].TargetPose()
		tp.x = 1240
		tp.z = -600
		tp.y = 0
		tp.rx = 0
		tp.ry = 3.141592
		tp.rz = 0
		tp.onlyRot = True
		self.proxy.go(tp)
		print 'a2'

	def b(self):
		print 'b1'
		tp = self.mods['RoboCompTrajectoryRobot2D'].TargetPose()
		tp.x = 0
		tp.z = 0
		tp.y = 0
		tp.rx = 0
		tp.ry = 3.141592
		tp.rz = 0
		tp.onlyRot = True
		self.proxy.go(tp)
		print 'b2'

