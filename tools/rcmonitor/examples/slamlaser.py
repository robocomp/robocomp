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
		arg = sys.argv
		arg.append("--Ice.MessageSizeMax=10240")
		self.ic = Ice.initialize(arg)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompSlamLaser'].SlamLaserPrx.checkedCast(self.prx)
		self.job()
		self.show()

	def job(self):
		self.mdata = self.proxy.getWholeGrid();

	def paintEvent(self, event):
		painter = QPainter(self)
		#print len(self.mdata[0].data), self.mdata[0].params.width, self.mdata[0].params.height, self.mdata[0].params.width*self.mdata[0].params.height
		self.qimage = QImage(self.mdata[0].data, self.mdata[0].params.width, self.mdata[0].params.height, QImage.Format_Indexed8)
		for i in range(256):
			self.qimage.setColor(i, QColor(i,i,i).rgb())
		#print self.qimage.valid(0, 0), self.qimage.valid(self.mdata[0].params.width-1, self.mdata[0].params.height-1), type(self.qimage)
		painter.drawImage(QRectF(0, 0, self.mdata[0].params.width, self.mdata[0].params.height), self.qimage)


