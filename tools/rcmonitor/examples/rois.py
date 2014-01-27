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
		self.proxy = self.mods['RoboCompRoimant'].RoimantPrx.checkedCast(self.prx)
		self.roiList = []
		self.job()
	def job(self):
		# Remote procedure call
		output = self.proxy.getROIList()
		# Set class copies
		self.roiList = output[0]
		self.bState = output[1].bState
		# Print number of ROIs
		print len(self.roiList)

	def paintEvent(self, event=None):
		xOff = self.width()/2.
		yOff = self.height()/2.
		xPos = 0
		yPos = 0
		div = 20.
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)
		# Draw grid
		for i in range(max(xOff,yOff)/30):
			x = 30*i
			painter.drawLine(xOff+x, 0, xOff+x, self.height())
			painter.drawLine(xOff-x, 0, xOff-x, self.height())
			painter.drawLine(0, yOff+x, self.width(), yOff+x)
			painter.drawLine(0, yOff-x, self.width(), yOff-x)
		# Draw ROIs
		painter.setPen(Qt.red)
		painter.setBrush(Qt.red)
		for roi in self.roiList:
			if not roi.casado: continue
			try:
				xPos = int((roi.z3D/div)+xOff-3)
				yPos = int((roi.x3D/div)+yOff-3)
			except:
				pass
			if type(xPos) == type(yPos) and type(xPos) == type(int()):
				try:
					painter.drawEllipse(int(xPos), int(yPos), 6, 6)
				except:
					print 'ROI :-(', int(xPos)
					print type(xPos)
					print type(int(xPos))
					print roi.x3D, roi.x3D
					traceback.print_stack()
		# Draw base
		painter.setPen(Qt.blue)
		painter.setBrush(Qt.blue)
		try:
			xPos = int( (self.bState.z/div)+xOff-9)
			yPos = int( (self.bState.x/div)+yOff-9)
			start = int(((-self.bState.alfa*180/math.pi)-180-20)*16)
		except:
			pass
		if type(xPos) == type(yPos) and type(xPos) == type(start) and type(xPos) == type(int()):
			try:
				painter.drawPie(xPos, yPos, 18, 18, start, 20*2*16)
			except:
				print 'BASE :-('
				print type(xPos-7)
				print self.bState.z, self.bState.x, self.bState.alfa

		painter.end()
		painter = None
