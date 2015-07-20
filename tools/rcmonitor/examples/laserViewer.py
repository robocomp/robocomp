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

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *

class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompLaser'].LaserPrx.checkedCast(self.prx)
		self.show()
		self.label = QLabel('pixel/mm ratio', self)
		self.label.move(10, 10)
		self.label.show()
		self.spinBox = QDoubleSpinBox(self)
		self.spinBox.show()
		self.spinBox.move(10, 10+self.label.height())
		self.spinBox.resize(150, 25)
		self.spinBox.setMaximum(1024.)
		self.spinBox.setMinimum(0.004)
		self.spinBox.setDecimals(6);
		self.spinBox.setSingleStep(0.00005)
		self.job()
		print self.proxy.getLaserConfData()

	def job(self):
		try:
			self.data, basura = self.proxy.getLaserAndBStateData()
			print '-----'
			print len(self.data), ' ',  basura.x, ' ', basura.z, ' ', basura.alpha
			print self.data[0]
			print self.data[-1]
		except:
			print 'No laser connection.'
		return None
	def measure2coord(self, measure):
		const_mul = self.spinBox.value()
		x = math.cos(measure.angle-0.5*math.pi)*measure.dist*const_mul+(0.5*self.width())
		y = math.sin(measure.angle-0.5*math.pi)*measure.dist*const_mul+(0.5*self.height())
		return x, y
	def paintEvent(self, event=None):
		xOff = self.width()/2.
		yOff = self.height()/2.
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)

		for point in self.data:
			newCoor = self.measure2coord(point)
			painter.drawRect(newCoor[0]-1, newCoor[1]-1, 2, 2)

		for wm in range(10):
			w = 1000. * (1.+wm) * self.spinBox.value()
			painter.drawEllipse(QRectF(0.5*self.width()-w/2., 0.5*self.height()-w/2., w, w))
		#painter.drawLine(QPoint(0.5*self.width(), 0.5*self.height()), QPoint(0.5*self.width(), 0.5*self.height()+20))
		#painter.drawLine(QPoint(0.5*self.width(), 0.5*self.height()), QPoint(0.5*self.width()+5, 0.5*self.height()+20))
		#painter.drawLine(QPoint(0.5*self.width(), 0.5*self.height()), QPoint(0.5*self.width()-5, 0.5*self.height()+20))

		painter.end()
		painter = None
