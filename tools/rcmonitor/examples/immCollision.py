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
		self.Lista=list();
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompInnerModelManager'].InnerModelManagerPrx.checkedCast(self.prx)

		self.x = 0.
		self.collided = False

		pose = self.mods['RoboCompInnerModelManager'].Pose3D()
		pose.x = 0
		pose.y = 200.
		pose.z = 0.
		pose.rx = 0.
		pose.ry = 0.
		pose.rz = 0.
		self.proxy.setPoseFromParent("caja2", pose)

		self.show()
	

	def job(self):
		pose = self.mods['RoboCompInnerModelManager'].Pose3D()
		pose.x = 500.*math.sin(self.x)
		if not self.collided:
			pose.y = 200.
		else:
			pose.y = 300.
		pose.z = 0.
		pose.rx = 0.
		pose.ry = 0.
		pose.rz = 0.
		self.proxy.setPoseFromParent("caja1", pose)
		self.collided = self.proxy.collide("cajaMesh1", "cajaMesh2")
		print (self.x, self.collided)

		self.x += 0.05

