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
		self.proxy = self.mods['RoboCompVision'].VisionPrx.checkedCast(self.prx)
		self.pyrList = []
		for level in range(4): self.pyrList.append(None)
		self.job()
	def job(self):
		#for level in range(len(self.pyrList)):
			#output = self.proxy.getLevelPyr(0, level)
			#self.pyrList[level] = output[0]
			output = self.proxy.getWholePyrRGBBothCam()
			for level in range(4):
				self.pyrList[level] = output[1][level]

	def paintEvent(self, event=None):
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)
		xPos = -160
		yPos = self.height()
		for level in range(len(self.pyrList)):
			xPos = xPos + 160/(2**level)
			yPos = yPos - 240/(2**level)
			qimage = QImage(self.pyrList[level], 320/(2**level), 240/(2**level), QImage.Format_RGB888);
			#qimage = QImage(self.pyrList[level], 320/(2**level), 240/(2**level), QImage.Format_Indexed8)
			#for i in range(256):
				#qimage.setColor(i, QColor(i,i,i).rgb())
			painter.drawImage(QPointF(xPos, yPos), qimage)

		painter.end()
		painter = None
