#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#    ------------------------
#    -----  AGGLEditor  -----
#    ------------------------
#
#    A free/libre open-source graph grammar drawing tool.
#
#    Copyright (C) 2012-2017 by Luis J. Manso
#
#    AGGLEditor is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    AGGLEditor is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with AGGLEditor. If not, see <http://www.gnu.org/licenses/>.

# Ctrl+c handling
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)
# Python distribution imports
import sys, traceback, os, re, threading, time, string, math
# Qt interface
import PySide
from PySide.QtCore import *
from PySide.QtGui import *
from PySide.QtSvg import *

import Image, ImageOps
import numpy as np

from inspect import currentframe, getframeinfo

sys.path.append('/usr/local/share/agm')
from ui_guiAGGLEditor import Ui_MainWindow
from ui_agglTypeEditor import Ui_TypeEditor
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
fontSize = 14


from AGMModule import *


import networkx as nx

import matplotlib
matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4'] = 'PySide'
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt4agg import ( FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
from matplotlib.figure import Figure
from PySide import QtGui, QtCore
import random

from weakref import proxy


class Plotter(FigureCanvas):
	def __init__(self, parent):
		''' plot some random stuff '''
		self.parent = proxy(parent)
		self.fig = Figure()
		super(Plotter,self).__init__(self.fig)
		# create an axis
		self.axes = self.fig.add_subplot(111)
	def plot(self, data):
		self.axes.clear()
		g = nx.DiGraph()
		for symbolType in data:
			g.add_node(symbolType)
			for d in data[symbolType]:
				g.add_edge(symbolType, d, weight=1.0)
		nodes = data.keys()
		labels = {}
		for n in nodes:
			labels[n] = n
		from networkx.drawing.nx_agraph import graphviz_layout
		# nx.draw(g, ax=self.axes, node_size=311, node_color='w', font_weight='bold', linewidths=0, font_size=18, nodelist=nodes, labels=labels)
		nx.draw(g, pos=graphviz_layout(g, prog='dot'), ax=self.axes, node_size=311, node_color='w', font_weight='bold', linewidths=0, font_size=18, nodelist=nodes, labels=labels)
		self.fig.canvas.draw()
	def binding_plotter_with_ui(self):
		self.parent.verticalLayout.insertWidget(2, self)

class TypeEditor(QWidget, Ui_TypeEditor):
	def __init__(self):
		QWidget.__init__(self)
		self.setupUi(self)
		self.plotter = Plotter(self)
		self.plotter.binding_plotter_with_ui()
	def plot(self, data):
		self.plotter.plot(data)

class AGMEditor(QMainWindow):
	def __init__(self, filePath=''):
		QMainWindow.__init__(self)
		self.filePath = filePath
		self.modified = False

		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		# Type Hierarchy
		self.typeHierarchyWidget = TypeEditor()

		cursorSize = 25
		self.cursor = {}
		self.cursor['add node'] = QCursor(QPixmap('/usr/local/share/agm/icons/nodeadd.png').scaled(cursorSize,cursorSize))
		self.cursor['remove node'] = QCursor(QPixmap('/usr/local/share/agm/icons/noderemove.png').scaled(cursorSize,cursorSize))
		self.cursor['rename node'] = QCursor(QPixmap('/usr/local/share/agm/icons/noderename.png').scaled(cursorSize,cursorSize))
		self.cursor['change type'] = QCursor(QPixmap('/usr/local/share/agm/icons/noderetype.png').scaled(cursorSize,cursorSize))
		self.cursor['move node'] = QCursor(QPixmap('/usr/local/share/agm/icons/nodemove.png').scaled(cursorSize,cursorSize))
		self.cursor['add edge'] = QCursor(QPixmap('/usr/local/share/agm/icons/linkadd.png').scaled(cursorSize,cursorSize))
		self.cursor['remove edge'] = QCursor(QPixmap('/usr/local/share/agm/icons/linkremove.png').scaled(cursorSize,cursorSize))
		self.cursor['change label'] = QCursor(QPixmap('/usr/local/share/agm/icons/linktype.png').scaled(cursorSize,cursorSize))
		self.cursor['negate edge'] = QCursor(QPixmap('/usr/local/share/agm/icons/linknegate.png').scaled(cursorSize,cursorSize))


		# Graph painters
		self.lhsPainter = GraphDraw(self.ui.lhsParentWidget, self, "LHS")
		self.rhsPainter = GraphDraw(self.ui.rhsParentWidget, self, "RHS")
		self.timer = QTimer()
		self.tool = ''
		self.statusBar().hide()
		self.connect(self.timer,                               SIGNAL('timeout()'),                    self.draw)
		# self.connect(self.ui.toolsList,                        SIGNAL('currentRowChanged(int)'),       self.selectTool)
		self.connect(self.ui.rulesList,                        SIGNAL('currentRowChanged(int)'),       self.changeRule)
		self.connect(self.ui.actionChangeAppearance,           SIGNAL("triggered(bool)"),              self.changeAppearance)
		self.connect(self.ui.actionTypeHierarchy,              SIGNAL("triggered(bool)"),              self.showTypeHierarchy)
		self.connect(self.ui.actionAddRule,                    SIGNAL("triggered(bool)"),              self.addRule)
		self.connect(self.ui.actionRemoveCurrentRule,          SIGNAL("triggered(bool)"),              self.removeCurrentRule)
		self.connect(self.ui.actionRenameCurrentRule,          SIGNAL("triggered(bool)"),              self.renameCurrentRule)
		self.connect(self.ui.actionExport,                     SIGNAL("triggered(bool)"),              self.exportRule)
		self.connect(self.ui.actionGenerateCode,               SIGNAL("triggered(bool)"),              self.generateCode)
		self.connect(self.ui.actionGenerateAGGLPlanner,        SIGNAL("triggered(bool)"),              self.generateAGGLPlannerCode)
		self.connect(self.ui.actionExportAllRules,             SIGNAL("triggered(bool)"),              self.exportAll)
		self.connect(self.ui.actionExportAllRulesPNG,          SIGNAL("triggered(bool)"),              self.exportAllPNG)
		self.connect(self.ui.actionSaveAs,                     SIGNAL("triggered(bool)"),              self.saveAs)
		self.connect(self.ui.actionSave,                       SIGNAL("triggered(bool)"),              self.save)
		self.connect(self.ui.actionOpen,                       SIGNAL("triggered(bool)"),              self.open)
		self.connect(self.ui.actionQuit,                       SIGNAL("triggered(bool)"),              self.appClose)
		self.connect(self.ui.actionGraphmar,                   SIGNAL("triggered(bool)"),              self.about)
		self.connect(self.ui.actionHideToolNames,              SIGNAL("triggered(bool)"),              self.hideToolNames)
		self.connect(self.ui.passiveCheckBox,                  SIGNAL("stateChanged(int)"),            self.changePassive)
		self.connect(self.ui.hierarchicalCheckBox,             SIGNAL("stateChanged(int)"),            self.changeHierarchical)
		self.connect(self.ui.cost,                             SIGNAL("valueChanged(int)"),            self.changeCost)



		self.ui.actionSaveAs.setShortcut(QKeySequence( Qt.CTRL + Qt.Key_S + Qt.Key_Shift))
		self.ui.actionSave.setShortcut(QKeySequence( Qt.CTRL + Qt.Key_S))
		self.ui.actionOpen.setShortcut(QKeySequence( Qt.CTRL + Qt.Key_O))
		self.ui.actionQuit.setShortcut(QKeySequence(Qt.CTRL + Qt.Key_Q))
		#self.ui.actionNextRule.setShortcut(QKeySequence(Qt.Key_PageDown))
		#self.ui.actionPrevRule.setShortcut(QKeySequence(Qt.Key_PageUp))

		#self.ui.splitter.setStyleSheet("QSplitter::handle { background-color: gray }")

		self.connect(self.ui.textParameters,    SIGNAL("textChanged()"), self.textParametersChanged)
		self.connect(self.ui.textPrecondition,  SIGNAL("textChanged()"), self.textPreconditionChanged)
		self.connect(self.ui.textEffect,        SIGNAL("textChanged()"), self.textEffectChanged)
		# self.connect(self.ui.comboRuleTextEdit, SIGNAL("textChanged()"),    self.comboTextChanged)


		self.shortcutDown = QShortcut(QKeySequence("PgDown"), self)
		self.shortcutUp   = QShortcut(QKeySequence("PgUp"  ), self)
		self.connect(self.shortcutDown, SIGNAL("activated()"), self.pgDown)
		self.connect(self.shortcutUp,   SIGNAL("activated()"), self.pgUp)


		self.connect(self.ui.addnodebutton, SIGNAL('toggled(bool)'), self.on_addnodebutton)
		self.connect(self.ui.removenodebutton, SIGNAL('toggled(bool)'), self.on_removenodebutton)
		self.connect(self.ui.renamenodebutton, SIGNAL('toggled(bool)'), self.on_renamenodebutton)
		self.connect(self.ui.changenodetypebutton, SIGNAL('toggled(bool)'), self.on_changenodetypebutton)
		self.connect(self.ui.nodemovebutton, SIGNAL('toggled(bool)'), self.on_nodemovebutton)
		self.connect(self.ui.addedgebutton, SIGNAL('toggled(bool)'), self.on_addedgebutton)
		self.connect(self.ui.removeedgebutton, SIGNAL('toggled(bool)'), self.on_removeedgebutton)
		self.connect(self.ui.changeedgelabelbutton, SIGNAL('toggled(bool)'), self.on_changeedgelabelbutton)
		self.connect(self.ui.negateedgebutton, SIGNAL('toggled(bool)'), self.on_negateedgebutton)


		self.connect(self.typeHierarchyWidget.typeList, SIGNAL("currentRowChanged(int)"), self.currentTypeChanged)
		# self.connect(self.typeHierarchyWidget.typeList, SIGNAL("currentItemChanged(QListWidgetItem)"), self.currentTypeChanged)
		# self.connect(self.typeHierarchyWidget.typeList, SIGNAL("itemPressed(QWidgetItem)"), self.currentTypeChanged)
		self.connect(self.typeHierarchyWidget.newButton, SIGNAL("clicked()"), self.createType)
		self.connect(self.typeHierarchyWidget.renameButton, SIGNAL("clicked()"), self.renameType)
		self.connect(self.typeHierarchyWidget.includeButton, SIGNAL("clicked()"), self.includeTypeInheritance)
		self.connect(self.typeHierarchyWidget.removeButton, SIGNAL("clicked()"), self.removeTypeInheritance)
		self.connect(self.typeHierarchyWidget.okButton, SIGNAL("clicked()"), self.typeHierarchyWidget.hide)

		self.hideToolNames(False)

		# Get settings
		settings = QSettings("AGM", "mainWindowGeometry")
		value = settings.value("geometry")
		if value != None:
			self.restoreGeometry(value)

		self.timer.start(150)
		# self.ui.toolsList.setCurrentRow(4)
		# self.selectTool(4)
		# self.ui.toolsList.setCurrentRow(4)
		# self.selectTool(4)

		global vertexDiameter
		global fontName
		global fontSize
		self.agmData = AGMFileData()
		if len(filePath)>0:
			self.openFromFile(filePath)
			try:
				vertexDiameter = self.agmData.properties['vertexDiameter']
			except:
				pass
			try:
				global nodeThickness
				nodeThickness = self.agmData.properties['nodeThickness']
			except:
				pass
			try:
				global lineThickness
				lineThickness = self.agmData.properties['lineThickness']
			except:
				pass
			try:
				global dashPattern
				dashPattern = []
			except:
				pass
			try:
				fontName = self.agmData.properties['fontName']
			except:
				pass
			try:
				fontSize = self.agmData.properties['fontSize']
			except:
				pass
		else:
			self.addRule()
			frameinfo = getframeinfo(currentframe())
			frameinfo = getframeinfo(currentframe())
		self.ui.rulesList.setCurrentRow(0)
		self.ui.rulesList.setFocus(Qt.OtherFocusReason)

		# Node appearance
		self.appearance = Appearance()
		self.appearance.ui.radius.setValue(vertexDiameter)


		# Font
		font = QFont(fontName, fontSize, weight=0, italic=False)
		font.setStyle(QFont.StyleNormal)
		self.fontDialog = QFontDialog(font, self)
		self.fontDialog.setCurrentFont(font)
		self.font = self.fontDialog.currentFont()
		self.connect(self.ui.actionChangeFont, SIGNAL("triggered(bool)"), self.changeFont)

		# Sizes
		self.show()
		sh = self.ui.centralwidget.height()
		self.ui.splitter.setSizes([int(0.65*sh), int(0.35*sh)])


		self.tool = 'move node'
		self.uncheckOtherTools(self.tool)

	def pgDown(self):
		r = self.ui.rulesList.currentRow()+1
		if r>=0 and r<self.ui.rulesList.count():
			self.ui.rulesList.setCurrentRow(r)
	def pgUp(self):
		r = self.ui.rulesList.currentRow()-1
		if r>=0 and r<self.ui.rulesList.count():
			self.ui.rulesList.setCurrentRow(r)

	def on_addnodebutton(self, v):
		if v:
			self.tool = 'add node'
			self.uncheckOtherTools(self.tool)
	def on_removenodebutton(self, v):
		if v:
			self.tool = 'remove node'
			self.uncheckOtherTools(self.tool)
	def on_renamenodebutton(self, v):
		if v:
			self.tool = 'rename node'
			self.uncheckOtherTools(self.tool)
	def on_changenodetypebutton(self, v):
		if v:
			self.tool = 'change type'
			self.uncheckOtherTools(self.tool)
	def on_nodemovebutton(self, v):
		if v:
			self.tool = 'move node'
			self.uncheckOtherTools(self.tool)
	def on_addedgebutton(self, v):
		if v:
			self.tool = 'add edge'
			self.uncheckOtherTools(self.tool)
	def on_removeedgebutton(self, v):
		if v:
			self.tool = 'remove edge'
			self.uncheckOtherTools(self.tool)
	def on_changeedgelabelbutton(self, v):
		if v:
			self.tool = 'change label'
			self.uncheckOtherTools(self.tool)
	def on_negateedgebutton(self, v):
		if v:
			self.tool = 'negate edge'
			self.uncheckOtherTools(self.tool)

	def uncheckOtherTools(self, tool):
		if tool != 'add node':
 			self.ui.addnodebutton.setChecked(False)
		if tool != 'remove node':
 			self.ui.removenodebutton.setChecked(False)
		if tool != 'rename node':
 			self.ui.renamenodebutton.setChecked(False)
		if tool != 'change type':
 			self.ui.changenodetypebutton.setChecked(False)
		if tool != 'move node':
 			self.ui.nodemovebutton.setChecked(False)
		if tool != 'add edge':
 			self.ui.addedgebutton.setChecked(False)
		if tool != 'remove edge':
 			self.ui.removeedgebutton.setChecked(False)
		if tool != 'change label':
 			self.ui.changeedgelabelbutton.setChecked(False)
		if tool != 'negate edge':
 			self.ui.negateedgebutton.setChecked(False)
		self.lhsPainter.setCursor(self.cursor[tool])
		self.rhsPainter.setCursor(self.cursor[tool])


	def hideToolNames(self, value):
		self.ui.addnodebutton.setIcon(QIcon('/usr/local/share/agm/icons/nodeadd.png'))
		self.ui.removenodebutton.setIcon(QIcon('/usr/local/share/agm/icons/noderemove.png'))
		self.ui.renamenodebutton.setIcon(QIcon('/usr/local/share/agm/icons/noderename.png'))
		self.ui.changenodetypebutton.setIcon(QIcon('/usr/local/share/agm/icons/noderetype.png'))
		self.ui.nodemovebutton.setIcon(QIcon('/usr/local/share/agm/icons/nodemove.png'))
		self.ui.addedgebutton.setIcon(QIcon('/usr/local/share/agm/icons/linkadd.png'))
		self.ui.removeedgebutton.setIcon(QIcon('/usr/local/share/agm/icons/linkremove.png'))
		self.ui.changeedgelabelbutton.setIcon(QIcon('/usr/local/share/agm/icons/linktype.png'))
		self.ui.negateedgebutton.setIcon(QIcon('/usr/local/share/agm/icons/linknegate.png'))

		if value:
			self.ui.addnodebutton.setText('')
			self.ui.removenodebutton.setText('')
			self.ui.renamenodebutton.setText('')
			self.ui.changenodetypebutton.setText('')
			self.ui.nodemovebutton.setText('')
			self.ui.addedgebutton.setText('')
			self.ui.removeedgebutton.setText('')
			self.ui.changeedgelabelbutton.setText('')
			self.ui.negateedgebutton.setText('')
		else:
			self.ui.addnodebutton.setText('add symbol')
			self.ui.removenodebutton.setText('remove symbol')
			self.ui.renamenodebutton.setText('rename symbol')
			self.ui.changenodetypebutton.setText('change symbol type')
			self.ui.nodemovebutton.setText('move symbol')
			self.ui.addedgebutton.setText('add link')
			self.ui.removeedgebutton.setText('remove link')
			self.ui.changeedgelabelbutton.setText('change link label')
			self.ui.negateedgebutton.setText('negate link')

	# Manages close events
	def appClose(self):
		if self.modified:
			self.close()
		else:
			self.close()
		self.close()
	def closeEvent(self, closeevent):
		settings = QSettings("AGM", "mainWindowGeometry")
		g = self.saveGeometry()
		settings.setValue("geometry", g)

	def about(self):
		QMessageBox.information(self, "About", "Active Graph Grammar Language Editor (AGGLEditor):\n https://github.com/ljmanso/AGM/wiki \n\n Icons by Alexander Madyankin and Roman Shamin\n(MIT license)")
	def draw(self):
		self.lhsPainter.graph.setColors(self.rhsPainter.graph, True)
		self.rhsPainter.graph.setColors(self.lhsPainter.graph, False)
		self.lhsPainter.update()
		self.rhsPainter.update()
		for r in range(len(self.agmData.agm.rules)):
			if type(r) == AGMRule:
				item = self.ui.rulesList.item(r)
				item.setText(self.agmData.agm.rules[r].name)
	def changePassive(self, passive):
		p = True
		if passive == Qt.Unchecked:
			p = False
		self.agmData.agm.rules[self.ui.rulesList.currentRow()].passive = p
	def changeHierarchical(self, hierarchical):
		h = True
		if hierarchical == Qt.Unchecked:
			h = False
		self.agmData.agm.rules[self.ui.rulesList.currentRow()].hierarchical = h
	def changeCost(self, v):
		self.agmData.agm.rules[self.ui.rulesList.currentRow()].cost = v
	def changeFont(self):
		self.fontDialog.show()
	def changeAppearance(self):
		self.appearance.show()
	def showTypeHierarchy(self):
		self.typeHierarchyWidget.show()
	def addRule(self):
		ddd = 'rule' + str(len(self.agmData.agm.rules))
		self.ui.rulesList.addItem(ddd)
		l = AGMGraph(side='L')
		r = AGMGraph(side='R')
		self.agmData.agm.addRule(AGMRule(ddd, l, r, False))
	def removeCurrentRule(self):
		pos = self.ui.rulesList.currentRow()
		self.agmData.agm.rules = self.agmData.agm.rules[:pos] + self.agmData.agm.rules[pos+1:]
		self.ui.rulesList.takeItem(pos)
	def renameCurrentRule(self):
		pos = self.ui.rulesList.currentRow()
		r = RuleRenamer(self.width()/2, self.height()/2, self.agmData.agm.rules[pos], self)
		r.setFocus(Qt.OtherFocusReason)
	def changeRule(self, ruleN):
		if type(self.agmData.agm.rules[ruleN]) in [AGMRule, AGMHierarchicalRule]:
			self.ui.label.show()
			self.ui.label_2.show()
			self.ui.spacee.show()
			self.ui.lhsParentWidget.show()
			self.ui.rhsParentWidget.show()
			self.ui.cost.show()
			self.ui.label_3.show()
			self.ui.passiveCheckBox.show()
			self.ui.hierarchicalCheckBox.show()
			# self.ui.comboRuleTextEdit.hide()
			# self.ui.comboRuleLabel.hide()
			# self.ui.label_5.show()
			# self.ui.toolsList.show()
			self.lhsPainter.graph = self.agmData.agm.rules[ruleN].lhs
			self.rhsPainter.graph = self.agmData.agm.rules[ruleN].rhs
			try:
				self.ui.textParameters.setText(self.agmData.agm.rules[ruleN].parameters.replace("\n\t\t", "\n").lstrip())
			except:
				print traceback.format_exc()
			try:
				self.ui.textPrecondition.setText(self.agmData.agm.rules[ruleN].precondition.replace("\n\t\t", "\n").lstrip())
			except:
				print traceback.format_exc()
			try:
				self.ui.textEffect.setText(self.agmData.agm.rules[ruleN].effect.replace("\n\t\t", "\n").lstrip())
			except:
				print traceback.format_exc()
		else:
			self.ui.label.hide()
			self.ui.label_2.hide()
			self.ui.spacee.hide()
			self.ui.lhsParentWidget.hide()
			self.ui.rhsParentWidget.hide()
			self.ui.cost.hide()
			self.ui.label_3.hide()
			self.ui.passiveCheckBox.hide()
			self.ui.hierarchicalCheckBox.hide()
			# self.ui.comboRuleTextEdit.show()
			# self.ui.comboRuleLabel.show()
			# self.ui.label_5.hide()
			# self.ui.toolsList.hide()
			#self.ui.tabWidget.setTabEnabled(1, False)
			# self.ui.comboRuleTextEdit.setPlainText(self.agmData.agm.rules[ruleN].text)
		self.disconnect(self.ui.passiveCheckBox,      SIGNAL("stateChanged(int)"), self.changePassive)
		self.disconnect(self.ui.hierarchicalCheckBox, SIGNAL("stateChanged(int)"), self.changeHierarchical)
		self.disconnect(self.ui.cost,                 SIGNAL("valueChanged(int)"), self.changeCost)
		if self.agmData.agm.rules[ruleN].passive:
			self.ui.passiveCheckBox.setChecked(True)
		else:
			self.ui.passiveCheckBox.setChecked(False)
		if isinstance(self.agmData.agm.rules[ruleN], AGMHierarchicalRule):
			self.ui.hierarchicalCheckBox.setChecked(True)
		else:
			self.ui.hierarchicalCheckBox.setChecked(False)
		self.ui.cost.setValue(int(self.agmData.agm.rules[ruleN].cost))
		self.connect(self.ui.passiveCheckBox,      SIGNAL("stateChanged(int)"), self.changePassive)
		self.connect(self.ui.hierarchicalCheckBox, SIGNAL("stateChanged(int)"), self.changeHierarchical)
		self.connect(self.ui.cost,                 SIGNAL("valueChanged(int)"), self.changeCost)

		currRule = self.ui.rulesList.currentItem().text()
		self.setWindowTitle('AGGLEditor - ['+currRule+']')
	def generateCode(self):
		path_pddl = QFileDialog.getSaveFileName(self, "Save FULL grammar (model verification) PDDL file as", "", "*.pddl")[0]
		if not path_pddl.endswith(".pddl"): path_pddl += ".pddl"
		self.agmData.generatePDDL(path_pddl, False)
		path_pddl = QFileDialog.getSaveFileName(self, "Save partial grammar (active rules) PDDL file as", "", "*.pddl")[0]
		if not path_pddl.endswith(".pddl"): path_pddl += ".pddl"
		self.agmData.generatePDDL(path_pddl, True)
	def generateAGGLPlannerCode(self):
		path_apy = QFileDialog.getSaveFileName(self, "Save FULL grammar (model verification) AGGLPlanner file as", "", "*.aggl.py")[0]
		if not path_apy.endswith(".aggl.py"): path_apy += ".aggl.py"
		self.agmData.generateAGGLPlannerCode(path_apy, False)
		path_apy = QFileDialog.getSaveFileName(self, "Save partial grammar (active rules) AGGLPlanner file as", "", "*.aggl.py")[0]
		if not path_apy.endswith(".aggl.py"): path_apy += ".aggl.py"
		self.agmData.generateAGGLPlannerCode(path_apy, True)
	def exportRule(self):
		path = str(QFileDialog.getSaveFileName(self, "Export rule", "", "*"))
		if path[-4:] == '.svg': path = path[:-4]
		pathLHS = path + 'LHS.png'
		self.lhsPainter.export(pathLHS)
		pathRHS = path + 'RHS.png'
		self.rhsPainter.export(pathRHS)
	def exportAll(self):
		path = str(QFileDialog.getExistingDirectory(self, "Export all rules", ""))
		self.exportAllPath(path)
	def exportAllPath(self, path):
		lhs = self.lhsPainter.graph
		rhs = self.rhsPainter.graph
		for i in range(len(self.agmData.agm.rules)):
			rule = self.agmData.agm.rules[i]
			self.lhsPainter.graph = rule.lhs
			self.lhsPainter.export(str(path)+'/rule'+str(i)+'_lhs.svg')
			self.rhsPainter.graph = rule.rhs
			self.rhsPainter.export(str(path)+'/rule'+str(i)+'_rhs.svg')
		self.lhsPainter.graph = lhs
		self.rhsPainter.graph = rhs
	def exportAllPNG(self):
		path = str(QFileDialog.getExistingDirectory(self, "Export all rules", ""))
		self.exportAllPathPNG(path)
	def exportAllPathPNG(self, path):
		lhs = self.lhsPainter.graph
		rhs = self.rhsPainter.graph
		for i in range(len(self.agmData.agm.rules)):
			rule = self.agmData.agm.rules[i]
			# Export LHS
			self.lhsPainter.graph = rule.lhs
			lhsPathPNG = str(path)+'/rule'+str(i)+'_lhs.png'
			self.lhsPainter.exportPNG(lhsPathPNG)
			# Export RHS
			self.rhsPainter.graph = rule.rhs
			rhsPathPNG = str(path)+'/rule'+str(i)+'_rhs.png'
			self.rhsPainter.exportPNG(rhsPathPNG)
			# Crop images similarly
			#crop = False
			crop = True
			if crop:
				#L
				lImage=Image.open(lhsPathPNG)
				lImage.load()
				lImageBox = self.getBoxImage(lImage)
				wL,hL = lImage.size
				#R
				rImage=Image.open(rhsPathPNG)
				rImage.load()
				rImageBox = self.getBoxImage(rImage)
				wR,hR = rImage.size
				# ----
				if lImageBox == None:
					lImageBox = rImageBox
				if rImageBox == None:
					rImageBox = lImageBox
				w = wL
				if w == None: w = wR
				h = hL
				if h == None: h = hR
				imageBox = self.getProperBox(lImageBox, rImageBox, w, h)
				lCropped=lImage.crop(imageBox)
				lCropped.save(lhsPathPNG)
				rCropped=rImage.crop(imageBox)
				rCropped.save(rhsPathPNG)
		self.lhsPainter.graph = lhs
		self.rhsPainter.graph = rhs

	def getBoxImage(self, image):
		invert_im = ImageOps.invert(image)
		return invert_im.getbbox()

	def getProperBox(self, a, b, w, h):
		c = h/2
		y1 = min(a[1], b[1])
		y2 = max(a[3], b[3])
		ret = (0, y1, w, y2)
		return ret

	def open(self):
		path = str(QFileDialog.getOpenFileName(self, "Export rule", "", "*.aggl")[0])
		self.openFromFile(path)
	def currentTypeChanged(self):
		print 'a'
		try:
			if self.typeHierarchyWidget.previousSelected != self.typeHierarchyWidget.typeList.selectedItems():
				self.reloadTypes()
			self.typeHierarchyWidget.previousSelected = self.typeHierarchyWidget.typeList.selectedItems()
		except:
			self.reloadTypes()
			self.typeHierarchyWidget.previousSelected = self.typeHierarchyWidget.typeList.selectedItems()
	def reloadTypes(self):
		try:
			selected = self.typeHierarchyWidget.typeList.currentItem().text()
		except:
			selected = None
		# list
		prevlist = sorted([item.text().encode('latin1') for item in self.typeHierarchyWidget.typeList.findItems('', QtCore.Qt.MatchContains)])
		currlist = sorted(self.agmData.agm.types.keys())
		# print 'P', prevlist
		# print 'C', currlist
		if prevlist != currlist:
			self.typeHierarchyWidget.typeList.clear()
			for tp in sorted(self.agmData.agm.types.keys()):
				self.typeHierarchyWidget.typeList.addItem(tp)

		if selected:
			# available
			print 'in parents'
			prevlist = sorted([item.text().encode('latin1') for item in self.typeHierarchyWidget.availableList.findItems('', QtCore.Qt.MatchContains)])
			currlist = sorted(self.agmData.getPossibleParentsFor(selected))
			if prevlist != currlist:
				self.typeHierarchyWidget.availableList.clear()
				for x in sorted(self.agmData.getPossibleParentsFor(selected)):
					self.typeHierarchyWidget.availableList.addItem(x)
			# selected
			prevlist = sorted([item.text().encode('latin1') for item in self.typeHierarchyWidget.selectedList.findItems('', QtCore.Qt.MatchContains)])
			currlist = sorted(self.agmData.getTypesDirect(selected))
			if prevlist != currlist:
				self.typeHierarchyWidget.selectedList.clear()
				for x in sorted(self.agmData.getTypesDirect(selected)):
					self.typeHierarchyWidget.selectedList.addItem(x)
		else:
			self.typeHierarchyWidget.availableList.clear()
			self.typeHierarchyWidget.selectedList.clear()

		self.typeHierarchyWidget.plot(self.agmData.agm.typesDirect)

	def createType(self):
		ty = 'newType createType'
		self.agmData.addType(ty)
		self.reloadTypes()
		row = 0
		while row < self.typeHierarchyWidget.typeList.count():
			if self.typeHierarchyWidget.typeList.item(row).text() == ty:
				break
			else:
				row += 1
		self.typeHierarchyWidget.typeList.setCurrentRow(row)
	def renameType(self):
		try:
			if len(self.typeHierarchyWidget.typeList.selectedItems()) == 0:
				ret = QMessageBox.warning(self.typeHierarchyWidget, self.tr("AGGLEditor warning"), self.tr("Please, select a type to modify"))
				return
			try:
				self.typeHierarchyWidget.edit.show()
			except:
				self.typeHierarchyWidget.edit = QLineEdit(self.typeHierarchyWidget.renameButton)
				self.connect(self.typeHierarchyWidget.edit, SIGNAL("returnPressed()"), self.renameTypeDone)
				self.typeHierarchyWidget.edit.show()
			finally:
				self.typeHierarchyWidget.edit.setFocus()
		finally:
			pass

	def renameTypeDone(self):
		text = self.typeHierarchyWidget.edit.text()
		self.typeHierarchyWidget.edit.hide()
		items = self.typeHierarchyWidget.typeList.selectedItems()
		print items, len(items)
		if text in self.agmData.agm.types.keys():
			ret = QMessageBox.warning(self.typeHierarchyWidget, self.tr("AGGLEditor warning"), self.tr("The type "+text+" already exists"))
			return
		if len(items) != 0:
			for i in self.typeHierarchyWidget.typeList.findItems(items[0].text(), 0):
				self.agmData.agm.renameType(items[0].text(), text)
				i.setText(text)
				self.reloadTypes()
	def includeTypeInheritance(self):
		try:
			selectedType = self.typeHierarchyWidget.typeList.selectedItems()[0].text()
		except:
			ret = QMessageBox.warning(self.typeHierarchyWidget, self.tr("AGGLEditor Warning"), self.tr("No type to modify was selected"))
			return
		try:
			selectedParent = self.typeHierarchyWidget.availableList.selectedItems()[0].text()
		except:
			ret = QMessageBox.warning(self.typeHierarchyWidget, self.tr("AGGLEditor Warning"), self.tr("No type to include as parent to "+selectedType+" was selected"))
			return
		print 'include', selectedType, selectedParent
		self.agmData.agm.includeTypeInheritance(selectedType, selectedParent)
		self.reloadTypes()
	def removeTypeInheritance(self):
		try:
			selectedType = self.typeHierarchyWidget.typeList.selectedItems()[0].text()
		except:
			traceback.print_exc()
			ret = QMessageBox.warning(self.typeHierarchyWidget, self.tr("AGGLEditor Warning"), self.tr("No type to modify was selected"))
			return
		try:
			selectedParent = self.typeHierarchyWidget.selectedList.selectedItems()[0].text()
		except:
			traceback.print_exc()
			ret = QMessageBox.warning(self.typeHierarchyWidget, self.tr("AGGLEditor Warning"), self.tr("No type to remove from the parent list of "+selectedType+" was selected"))
			return
		print 'remove', selectedType, selectedParent
		self.agmData.agm.removeTypeInheritance(selectedType, selectedParent)
		self.reloadTypes()
	def openFromFile(self, path):
		if path[-5:] != '.aggl': path = path + '.aggl'
		self.agmData = AGMFileDataParsing.fromFile(path, verbose=False, includeIncludes=False)

		self.ui.rulesList.clear()
		for rule in self.agmData.agm.rules:
			q = QListWidgetItem()
			q.setText(rule.name)
			self.ui.rulesList.addItem(q)
			if type(rule) == AGMRule:
				pass

		self.reloadTypes()

	def saveAs(self):
		path = QFileDialog.getSaveFileName(self, "Save as", "", "*.aggl")[0]
		self.save(False, path)
	def save(self, triggered=False, path=''):
		if path == '':
			if self.filePath != '':
				path = self.filePath
			else:
				path = QFileDialog.getSaveFileName(self, "Save as", "", "*.aggl")[0]
		self.filePath = path
		self.agmData.properties['name'] = path.split('/')[-1].split('.')[0]
		global vertexDiameter
		self.agmData.properties['vertexDiameter'] = vertexDiameter
		global nodeThickness
		self.agmData.properties['nodeThickness'] = nodeThickness
		global lineThickness
		self.agmData.properties['lineThickness'] = lineThickness
		global longPattern
		self.agmData.properties['longPattern'] = longPattern
		global shortPattern
		self.agmData.properties['shortPattern'] = shortPattern
		global spacePattern
		self.agmData.properties['spacePattern'] = spacePattern
		global fontName
		self.agmData.properties['fontName'] = fontName
		global fontSize
		self.agmData.properties['fontSize'] = fontSize

		self.agmData.toFile(path)
		self.modified = False

	def textParametersChanged(self):
		self.agmData.agm.rules[self.ui.rulesList.currentRow()].parameters   = str(self.ui.textParameters.toPlainText().encode(  'latin1')).strip().replace("\n", "\n\t\t")
	def textPreconditionChanged(self):
		self.agmData.agm.rules[self.ui.rulesList.currentRow()].precondition = str(self.ui.textPrecondition.toPlainText().encode('latin1')).strip().replace("\n", "\n\t\t")
	def textEffectChanged(self):
		self.agmData.agm.rules[self.ui.rulesList.currentRow()].effect       = str(self.ui.textEffect.toPlainText().encode(      'latin1')).strip().replace("\n", "\n\t\t")

if __name__ == '__main__':
	app = QApplication(sys.argv)
	if len(sys.argv)>1:
		if len(sys.argv)>5:
			inputFile    = sys.argv[1]
			compileFlag1 = sys.argv[2]
			outputFile1  = sys.argv[3]
			compileFlag2 = sys.argv[4]
			outputFile2  = sys.argv[5]
			if compileFlag1 == '-f':
				agmData = AGMFileDataParsing.fromFile(inputFile, verbose=False)
				agmData.generatePDDL(outputFile1, False)
			if compileFlag2 == '-p':
				agmData = AGMFileDataParsing.fromFile(inputFile, verbose=False)
				agmData.generatePDDL(outputFile2, True)
		else:
			clase = AGMEditor(sys.argv[1])
			clase.show()
			app.exec_()
	else:
			clase = AGMEditor()
			clase.show()
			app.exec_()
