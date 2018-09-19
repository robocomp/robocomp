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
		arg = sys.argv
		arg.append("--Ice.MessageSizeMax=10240")
		self.ic = Ice.initialize(arg)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompCameraBus'].CameraBusPrx.checkedCast(self.prx)
		self.show()

		try:
			print self.proxy.getBusParams()
		except Ice.Exception:
			print "CameraBusComp not found"
			traceback.print_exc()

		self.comboLabel = QLabel("Camera")
		self.combo = QComboBox()
		self.connect(self.combo,SIGNAL('currentIndexChanged(int)'),self.cameraChanged)
		try:
			self.cameras = self.proxy.getAllCameraParams()
		except Ice.Exception:
			print 'CameraBus: Error: No cameras.'
			traceback.print_exc()
			
		print 'Cameras: ',
		for item in self.cameras:
			print item
			self.combo.addItem(item.name)
		self.combo.addItem("all")
		
		self.cSizeLabel = QLabel("Size")
		self.cSize = QComboBox()
		self.cSize.addItem("640x480")
		self.cSize.addItem("320x240")
		self.connect(self.cSize,SIGNAL('currentIndexChanged(int)'),self.sizeChanged)
		self.cModeLabel = QLabel("Mode")
		self.cMode = QComboBox()
		self.cMode.addItem("RGB")
		self.cMode.addItem("GREY")
		self.connect(self.cMode,SIGNAL('currentIndexChanged(int)'),self.modeChanged)

		self.layout = QVBoxLayout()
		self.layout.addWidget(self.comboLabel)
		self.layout.addWidget(self.combo)
		self.layout.addWidget(self.cModeLabel)
		self.layout.addWidget(self.cMode)
		self.layout.addWidget(self.cSizeLabel)
		self.layout.addWidget(self.cSize)
		self.hlayout = QHBoxLayout()
		self.hlayout.addLayout(self.layout)
		self.hlayout.addSpacerItem(QSpacerItem(10,10,QSizePolicy.Expanding,QSizePolicy.Expanding))

		self.frame = QFrame(self)
		self.frame.setFixedSize(320,240)
		self.vlayout = QVBoxLayout(self)
		self.vlayout.addWidget(self.frame)
		self.vlayout.addLayout(self.hlayout)

		self.setMinimumSize(320, 400)
		self.format = self.mods['RoboCompCameraBus'].Format()
		self.cameraChanged()
		self.modeChanged()
		self.sizeChanged()

		self.cameraList = []
		self.ctable=[]
		for i in range(0,255):
			self.ctable.append(qRgb ( i,i,i ))
		self.job()

	def job(self):
		try:
			if(self.combo.currentText() != "all"):
				self.image = self.proxy.getImage(self.camera, self.format)
			else:
				self.imageList = self.proxy.getSyncImages(self.cameraList,self.format,True)
		except Ice.Exception:
			print 'CameraBus: Error: Image not available.'
			traceback.print_exc()

	def paintEvent(self, event=None):
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)
		if(self.combo.currentText() == "all"):
			i = 0
			for image in self.imageList:
				if(self.format.modeImage == self.mods['RoboCompCameraBus'].Mode.GRAY8):
					qimage = QImage(image.data, image.frmt.width, image.frmt.height, QImage.Format_Indexed8)
					qimage.setColorTable( self.ctable)
				else:
					qimage = QImage(image.data,image.frmt.width,image.frmt.height,QImage.Format_RGB888)
				painter.drawImage(QPointF(self.frame.frameRect().x()+image.frmt.width*i,self.frame.frameRect().y()), qimage)
				i = i + 1
		else:
			if(self.format.modeImage == self.mods['RoboCompCameraBus'].Mode.GRAY8):
				qimage = QImage(self.image.data, self.image.frmt.width, self.image.frmt.height, QImage.Format_Indexed8)
				qimage.setColorTable ( self.ctable )
			else:
				qimage = QImage(self.image.data,self.image.frmt.width,self.image.frmt.height,QImage.Format_RGB888)
		
			painter.drawImage(QPointF(self.frame.frameRect().x(),self.frame.frameRect().y()), qimage)
		painter.end()


	def modeChanged(self):
		if(self.cMode.currentText() == "RGB"):
			self.format.modeImage = self.mods['RoboCompCameraBus'].Mode.RGB888Packet
		else:
			self.format.modeImage = self.mods['RoboCompCameraBus'].Mode.GRAY8
	
	def sizeChanged(self):
		temp = 1 
		if(self.combo.currentText() == "all"):
			temp = len(self.cameras)
		if(self.cSize.currentText() == "640x480"):
			self.format.width = 640
			self.format.height = 480
			self.frame.setFixedSize(self.format.width*temp,self.format.height)
			
		else:
			self.format.width = 320
			self.format.height = 240
			self.frame.setFixedSize(self.format.width*temp,self.format.height)
		self.job()
	def cameraChanged(self):
		self.camera = str(self.combo.currentText())



