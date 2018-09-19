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
		print 'Endpoint', endpoint
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompRoimant'].RoimantPrx.checkedCast(self.prx)
		self.leftPyrList = []
		self.rightPyrList = []
		for level in range(4):
			self.leftPyrList.append(None)
			self.rightPyrList.append(None)
		self.wdth = self.proxy.getRoiParams().width
		self.hght = self.proxy.getRoiParams().height
		self.job()
	def job(self):
		output = self.proxy.getBothPyramidsRGBAndLeftROIList()
		pos=0
		size=self.wdth*self.hght*3
		for level in range(4):
			self.leftPyrList[level] = output[0][pos:pos+size]
			self.rightPyrList[level] = output[2][pos:pos+size]
			pos = pos + size
			size = size/4

	def paintEvent(self, event=None):
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)
		xPos = -self.wdth/2
		yPos = self.height()
		for level in range(len(self.leftPyrList)):
			xPos = xPos + (self.wdth/2)/(2**level)
			yPos = yPos - self.hght/(2**level)
			qimage = QImage(self.leftPyrList[level], self.wdth/(2**level), self.hght/(2**level), QImage.Format_RGB888);
			painter.drawImage(QPointF(xPos, yPos), qimage)
			qimage = QImage(self.rightPyrList[level], self.wdth/(2**level), self.hght/(2**level), QImage.Format_RGB888);
			painter.drawImage(QPointF(xPos+self.wdth, yPos), qimage)

		painter.end()
		painter = None
