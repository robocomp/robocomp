#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  -----------------------
#  -----  GraphViewer  -----
#  -----------------------
#
#  A libre graph grammar drawing tool.
#
#    Copyright (C) 2012-2013 by Luis J. Manso
#
#    Graphmar is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    Graphmar is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with Graphmar. If not, see <http://www.gnu.org/licenses/>.

# Ctrl+c handling
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)
# Python distribution imports
import sys, traceback, os, re, threading, time, string, math
# Qt interface
from PySide.QtCore import *
from PySide.QtGui import *
from PySide.QtSvg import *
#from PySide.Qt import *


import Image, ImageOps
import numpy as np

from inspect import currentframe, getframeinfo

sys.path.append('/usr/local/share/agm/')
from ui_guiGraphViewer import Ui_MainWindow
from ui_appearance import Ui_Appearance
from parseAGGL import *

global vertexDiameter
vertexDiameter = 40
global nodeThickness
nodeThickness = 2.5
global lineThickness
lineThickness = 2.5
global longPattern
longPattern = 3
global shortPattern
shortPattern = 1
global spacePattern
spacePattern = 3
global dashPattern
dashPattern = []
global fontName
fontName = "Arial"
global fontSize
fontSize = 9


from AGMModule import *
import xmlModelParser

class GraphViewer(QMainWindow):
	def __init__(self, fileList):
		QMainWindow.__init__(self)
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		# Graph painters
		self.tool = 'Node - Move'
		self.connect(self.ui.actionChangeAppearance, SIGNAL("triggered(bool)"),  self.changeAppearance)
		self.connect(self.ui.actionQuit,             SIGNAL("triggered(bool)"),  self.appClose)
		self.connect(self.ui.actionAutomatic,        SIGNAL("triggered(bool)"),  self.setAutomaticLayout)
		self.connect(self.ui.actionManual,           SIGNAL("triggered(bool)"),  self.setManualLayout)
		self.drawers = []
		self.widgets = []
		self.automatic = False#True
		
		global vertexDiameter
		global fontName
		global fontSize
		vertexDiameter = 45
		fontName = 'Arial 14'

		# Node appearance
		self.appearance = Appearance()
		self.appearance.ui.radius.setValue(vertexDiameter)

		# Font
		font = QFont(fontName, fontSize)
		font.setItalic(False)
		font.setItalic(False)
		self.fontDialog = QFontDialog(font, self)
		self.fontDialog.setCurrentFont(font)
		self.font = self.fontDialog.currentFont()
		self.connect(self.ui.actionChangeFont,                 SIGNAL("triggered(bool)"),                                      self.changeFont)

		self.resize(1200,700)
		for fil in range(len(fileList)):
			self.widgets.append(QWidget())
			self.widgets[fil].resize(1200/len(fileList), 700)
			self.widgets[fil].show()
			self.ui.horizontalLayout.addWidget(self.widgets[fil])
		for fil in range(len(fileList)):
			self.drawers.append(GraphDraw(self.widgets[fil], self, "xxxx"))
			print 'xmlModelParser()', fileList[fil]
			self.drawers[fil].graph = xmlModelParser.graphFromXMLFile(fileList[fil])
			
			print 'nodes',self.drawers[fil].graph.nodes
			for key in self.drawers[fil].graph.nodes.keys():
				v = self.drawers[fil].graph.nodes[key]
				print key,v, self.drawers[fil].graph.nodes[key].attributes
				
			print 'links',self.drawers[fil].graph.links
			L=self.drawers[fil].graph.links
			for index, item in enumerate(L):
					print index, item
					print item.attributes
			self.drawers[fil].show()

		self.timer = QTimer()
		self.connect(self.timer, SIGNAL('timeout()'), self.draw)
		self.timer.start(20)

	def appClose(self):
		#if self.modified:
			#self.close()
		#else:
			#self.close()
		self.close()

	def setAutomaticLayout(self):
		self.automatic = True
	def setManualLayout(self):
		self.automatic = False
		#for fil in range(len(self.drawers)):
			#self.drawers[fil].setRandomly()
	# Manages close events
	def closeEvent(self, closeevent):
		settings = QSettings("AGM", "mainWindowGeometry");
		g = self.saveGeometry()
		settings.setValue("geometry", g)

	def about(self):
		QMessageBox.information(self, "About", "Active Grammar-based Modeling:\nhttps://github.com/ljmanso/AGM/wiki")

	def draw(self):
		for d in self.drawers:
			d.update()
			if self.automatic:
				for i in range(1):
					d.iterateSpring()

	def changePassive(self, passive):
		p = True
		if passive == Qt.Unchecked:
			p = False
		self.agmData.agm.rules[self.ui.rulesList.currentRow()].passive = p

	def selectTool(self, tool):
		self.tool = str(self.ui.toolsList.item(tool).text())

	def changeFont(self):
		self.fontDialog.show()

	def changeAppearance(self):
		self.appearance.show()

if __name__ == '__main__':
	app = QApplication(sys.argv)
	if len(sys.argv)>1:
		clase = GraphViewer(sys.argv[1:])
		clase.show()
		app.exec_()
	else:
		print 'Usage:\n\t'+sys.argv[0]


