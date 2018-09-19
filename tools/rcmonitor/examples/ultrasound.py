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

from PySide.QtCore import *
from PySide.QtGui import *

class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompUltrasound'].UltrasoundPrx.checkedCast(self.prx)
		self.show()

		self.combo = QComboBox(self)
		self.combo.show()
		self.combo.resize(200, 50)
		self.combo.move(10,10)

		self.sensors = self.proxy.getAllSensorParams()
		print 'Sensors: ',
		if len(self.sensors)==0: print 'UltraSound: Error: No sensors.'
		for item in self.sensors:
			print item.name
			self.combo.addItem(item.name)
		

		self.lcdNumber = QLCDNumber(self)
		self.lcdNumber.show()
		self.lcdNumber.move(10,self.combo.y()+self.combo.height())
		self.lcdNumber.resize(200,50)
		
		self.job()

	def job(self):
		self.distance = self.proxy.getSensorDistance(str(self.combo.currentText()))
		self.lcdNumber.display(self.distance)
		
		


