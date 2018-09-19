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
from ui_rgbdbusDlg import Ui_RGBDBusDlg


class C(QWidget):
	def __init__(self, endpoint, modules):
		print modules.keys()
		QWidget.__init__(self)
		print "init"
		self.ui = Ui_RGBDBusDlg()
		self.ui.setupUi(self)

		self.t = 0.
		arg = sys.argv
		arg.append("--Ice.MessageSizeMax=2000000")
		self.ic = Ice.initialize(arg)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		print endpoint
		print self.mods.keys()
		#self.proxy = self.mods['RoboCompRGBD'].RGBDPrx.checkedCast(self.prx)
		self.proxy = self.mods['RoboCompRGBDBus'].RGBDBusPrx.checkedCast(self.prx)
		self.paramsMap= self.proxy.getAllCameraParams()
		self.listCamera=[]
		for n in self.paramsMap:
			print 'name Camera: '+ n
			self.listCamera.append(n)

		self.nameActive=self.listCamera.pop()
		self.show()

		#To draw depth image. Maximum device depth. Now in milimeters
		self.maxDepth = 10000.0
		self.mySlot()
		self.myTimer = QTimer()
		self.myTimer.start(10000)
		self.connect(self.myTimer, SIGNAL('timeout()'), self.mySlot)

		self.job()

	def mySlot (self):
		if (len(self.listCamera) ==0):
			print "refill"
			for n in self.paramsMap:
				self.listCamera.append(n)
		self.nameActive=self.listCamera.pop()

	def job(self):
		print '--------------------'
		try:
			print 'Active camera:', self.nameActive
			self.cameralist = [self.nameActive]
			self.imagemap = self.proxy.getImages(self.cameralist)
			print self.imagemap
			self.ui.nameRGBD.setText('RGBD name: ' + self.nameActive)
			return
			print self.imagemap.keys()
			self.depth = self.imagemap[self.nameActive].depthImage
			self.color = self.imagemap[self.nameActive].colorImage
			if len(self.color) == 0:
				print 'Error retrieving rgb image (zero length)'
			if len(self.depth) == 0:
				print 'Error retrieving depth image (zero length)'
		except Ice.Exception:
			traceback.print_exc()

	def paintEvent(self, event=None):
		return
		print "paint Event"
		try:
			print len(self.color)
			print len(self.depth)
		except:
			print "not yet"
		print 3*( (640*480)/2)
		if (len(self.color) == 3*640*480 or len(self.depth) == 640*480):
			w=640
			h=480
		elif (len(self.color) == 3*320*240 or len(self.depth) == (640*480)/2):
			w=320
			h=240
		else:
			print 'we shall not paint!'
			return
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)

		v = ''
		for i in range(len(self.depth)):
			ascii = 0
			try:
				ascii = int((self.depth[i] / self.maxDepth ) * 256)
			except:
				pass
			if ascii > 255: ascii = 255
			v += chr(ascii)
		image = QImage(self.color, w, h, QImage.Format_RGB888)
		imageGrey = QImage(v, w, h, QImage.Format_Indexed8)
		for i in range(256):
			imageGrey.setColor(i, QColor(i,i,i).rgb())
		painter.drawImage(QPointF(self.ui.frameRGB.x(), self.ui.frameRGB.y()), image)
		painter.drawImage(QPointF(self.ui.frame.x(), self.ui.frame.y()), imageGrey)
		painter.end()
		painter = None
