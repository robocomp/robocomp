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

import Ice, sys, math, traceback,os

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		arg = sys.argv
		self.ic = Ice.initialize(arg)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompQrdecode'].QrdecodePrx.checkedCast(self.prx)
		self.show()
	
		self.image = QImage(self)
		self.ledit = QLineEdit(self)

		self.frame = QFrame(self)
		self.frame.setFixedSize(320,240)

		self.pbSend = QPushButton("Send",self)
		self.pbSend.show()
		self.connect(self.pbSend,SIGNAL('clicked()'),self.send)

		self.pbLoad = QPushButton("Load",self)
		self.pbLoad.show()
		self.connect(self.pbLoad,SIGNAL('clicked()'),self.load)
		
		self.hlayout = QHBoxLayout()
		self.hlayout.addSpacerItem(QSpacerItem(10,10,QSizePolicy.Expanding,QSizePolicy.Expanding))
		self.hlayout.addWidget(self.pbLoad)
		self.hlayout.addWidget(self.pbSend)

		self.layout = QVBoxLayout(self)
		self.layout.addWidget(self.frame)
		self.layout.addLayout(self.hlayout)
		self.layout.addWidget(self.ledit)

	def job(self):
		pass
		
	def paintEvent(self, event=None):
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)
		self.frame.setFixedSize(self.image.width(),self.image.height())
		painter.drawImage(QPointF(self.frame.frameRect().x(),self.frame.frameRect().y()), self.image)
		painter.end()

	def send(self):
		try:
			if(self.image.width() > 0 and self.image.height > 0):
				image = self.image.convertToFormat(QImage.Format_RGB888)
				bytes = image.bits().asstring(image.numBytes())
				message = self.proxy.decode(bytes,image.width(),image.height())
				self.ledit.setText(message)
				
		except Ice.Exception:
			treceback.print_exc()
	  
	def load(self):
		filename = QFileDialog.getOpenFileName(self, str("Load image"),str(os.environ['HOME']),str("Images (*.png *.gif *.jpg)"))
		print (filename)
		self.image.load(filename)




