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
import random

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompSpeech'].SpeechPrx.checkedCast(self.prx)
		self.show()

		self.ledit = QLineEdit(self)
		self.ledit.resize(320,40)
		self.ledit.show()
	
		self.commandButton = QPushButton("Say", self)
		self.commandButton.resize(200, 50)
		self.commandButton.show()
		self.commandButton.move(0,self.ledit.y()+self.ledit.height())
		self.connect(self.commandButton, SIGNAL('clicked()'), self.clicked )

		self.randomButton = QPushButton("Random", self)
		self.randomButton.resize(200, 50)
		self.randomButton.show()
		self.randomButton.move(0,self.commandButton.y()+self.commandButton.height())
		self.connect(self.randomButton, SIGNAL('clicked()'), self.random )
		
#		self.cbType = QComboBox(self)
#		self.cbType.addItem("Felipe")
#		self.cbType.addItem("Random")
#		self.cbType.addItem("Binary")
#		self.cbType.show()
#		self.connect(self.cbType,SIGNAL('currentIndexChanged(int)'),self.typeModified)
		
		
		self.hlayout = QHBoxLayout(self)
		self.hlayout.addWidget(self.ledit)
		self.hlayout.addWidget(self.commandButton)
		self.hlayout.addWidget(self.randomButton)
#		self.hlayout.addWidget(self.cbType)
		
		
		self.setMinimumSize(210, 180)
		#self.job()

	def job(self):
		pass
		#self.clicked()

	def clicked(self):
		try:
			texto = self.ledit.text()
			print(texto)
			self.proxy.say(texto,True)
		except Ice.Exception:
			treceback.print_exc()

	def typeModified(self,mode):
		try:
			self.proxy.selectMouthMovement(mode)
		except Ice.Exception:
			treceback.print_exc()

	def random(self):
		try:
			file = open("frases.txt")
			texto=""	
			random.seed()
			for i in range(1,random.randint(1,26)):
				texto=file.readline()
			file.close()
			#texto =  texto, "utf-8" )
			print (texto)
			self.proxy.say(texto,True)
		except Ice.Exception:
			traceback.print_exc()

