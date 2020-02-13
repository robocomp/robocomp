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

		self.included = False

		self.button = QPushButton("reset", self)
		self.button.move(20,20)
		self.connect(self.button, SIGNAL("clicked()"), self.buttonSlot)
		self.button.show()
		

		self.show()

	def buttonSlot(self):
		if self.included:
			print ("Removing")
			self.remove()
			print ("Removed")
			self.included = False
		else:
			print ("Including")
			self.include()
			print ("Included")
			self.included = True

	def include(self):
		joint = self.mods['RoboCompInnerModelManager'].jointType()
		joint.lx = -1
		joint.ly = -1
		joint.lz = -1
		joint.hx = -1
		joint.hy = -1
		joint.hz = -1
		joint.pose.x = 0
		joint.pose.y = 0
		joint.pose.z = 0
		joint.pose.rx = 0
		joint.pose.ry = 0
		joint.pose.rz = 0
		joint.mass = 0.0001
		joint.port = 6969
		joint.min = -1
		joint.max = 1
		joint.axis = "z"

		self.proxy.addJoint("lalala", "world", joint);
		
	def remove(self):
		self.proxy.removeNode("lalala");
		

	def job(self):
		pass

