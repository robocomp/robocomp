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
from PySide2.QtWidget import *


class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		print ('Endpoint', endpoint)
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompTracker'].TrackerPrx.checkedCast(self.prx)
		self.pyrList = []
		for level in range(6): self.pyrList.append(None)
		self.nLevels = 6
		self.job()
	def job(self):
		output = self.proxy.getTrackingData()
		for level in range(len(output[0])):
			self.pyrList[level] = output[0][level]
		self.nLevels = output[1]
		self.pointsList = output[2]
		self.sizeList = output[3]
		
		output = self.proxy.getTrackerState()
		print ('TrackerState', output)
		#if output==self.proxy.TrackerState.NoTarget:
			#print 'TrackerState: NoTarget'
		#elif output=='RoboCompTracker::LostTarget':
			#print 'TrackerState: LostTarget'
		#else:
			#print 'TrackerState: Tracking'

	def paintEvent(self, event=None):
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)
		xPos = -160
		yPos = self.height()
		for level in range(self.nLevels):
			xPos = xPos + 160/(2**level)
			yPos = yPos - 240/(2**level)
			qimage = QImage(self.pyrList[level], 320/(2**level), 240/(2**level), QImage.Format_Indexed8)
			for i in range(256):
				qimage.setColor(i, QColor(i,i,i).rgb())
			painter.drawImage(QPointF(xPos, yPos), qimage)
			painter.setPen(QColor("red"))
			if level < len(self.pointsList):
				for i in range(len(self.pointsList[level])):
					painter.drawRect(QRect(QPoint(self.pointsList[level][i].x-self.sizeList[level].w/2+xPos, self.pointsList[level][i].y-self.sizeList[level].h/2+yPos), QSize(self.sizeList[level].w, self.sizeList[level].h)))

		painter.end()
		painter = None
