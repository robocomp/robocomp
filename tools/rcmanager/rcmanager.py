#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  -----------------------
#  -----  rcmanager  -----
#  -----------------------
#  An ICE component manager.
#
#    Copyright (C) 2009-2015 by RoboLab - University of Extremadura
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
#


#
# CODE BEGINS
#
import sys, time, traceback, os, math, random, threading, time
import Ice

from PyQt4 import QtCore, QtGui, Qt
from ui_formManager import Ui_Form

import rcmanagerConfig

global_ic = Ice.initialize(sys.argv)

# Ctrl+c handling
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)


dict = rcmanagerConfig.getDefaultValues()
initDir = os.getcwd()

sys.path.append('.')
sys.path.append('/opt/robocomp/bin')

import rcmanagerEditor


# CommandDialog class: It's the dialog sending "up()/down() component X signal to the main
class CommandDialog(QtGui.QWidget):
	def __init__(self, parent, x, y):
		QtGui.QWidget.__init__(self)
		self.setParent(parent)
		self.setGeometry(x, y, 100, 75)
		self.button1 = QtGui.QPushButton(self)
		self.button1.setGeometry(0, 0, 100, 25)
		self.button1.setText('up')
		self.button2 = QtGui.QPushButton(self)
		self.button2.setGeometry(0, 25, 100, 25)
		self.button2.setText('down')
		self.button3 = QtGui.QPushButton(self)
		self.button3.setGeometry(0, 50, 100, 25)
		self.button3.setText('edit config')
		self.show()
		self.connect(self.button1, QtCore.SIGNAL('clicked()'), self.but1)
		self.connect(self.button2, QtCore.SIGNAL('clicked()'), self.but2)
		self.connect(self.button3, QtCore.SIGNAL('clicked()'), self.but3)
	def but1(self): self.emit(QtCore.SIGNAL("up()"))
	def but2(self): self.emit(QtCore.SIGNAL("down()"))
	def but3(self): self.emit(QtCore.SIGNAL("config()"))


# ComponentChecker class: Threaded endpoint-pinging class.
class ComponentChecker(threading.Thread):
	def __init__(self, endpoint):
		threading.Thread.__init__(self)
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.daemon = True
		self.reset()
		self.exit = False
		self.alive = False
		self.aPrx = None
		try:
			self.aPrx = global_ic.stringToProxy(endpoint)
			self.aPrx.ice_timeout(1)
		except:
			print "Error creating proxy to " + endpoint
			if len(endpoint) == 0:
				print 'please, provide an endpoint'
			raise
			
	def run(self):
		global global_ic
		while self.exit == False:
			try:
				self.aPrx.ice_ping()
				self.mutex.lock()
				self.alive = True
				self.mutex.unlock()
			except:
				self.mutex.lock()
				self.alive = False
				self.mutex.unlock()
			time.sleep(0.5)
	def reset(self):
		self.mutex.lock()
		self.alive = False
		self.mutex.unlock()
	def isalive(self):
		self.mutex.lock()
		r = self.alive
		self.mutex.unlock()
		return r
	def stop(self):
		self.exit = True
	def runrun(self):
		if not self.isAlive(): self.start()

#
# Application main class.
#
class TheThing(QtGui.QDialog):
	def __init__(self):
		# Create a component checker
		self.componentChecker = {}
		self.configFile = os.path.expanduser('~/rcmanager.xml')
		# Gui config
		global dict
		QtGui.QDialog.__init__(self)
		self.root = '/opt/robocomp/'
		self.ui = Ui_Form()
		self.ui.setupUi(self)
		self.canvas = GraphView(self.ui.graphTab)
		self.canvas.setGeometry(0, 0, 531, 581)
		self.canvas.show()
		self.canvasTimer = QtCore.QTimer()
		self.canvasFastTimer = QtCore.QTimer()
		self.connect(self.canvas, QtCore.SIGNAL("nodeReleased()"), self.setFastState)
		self.setFastState(True)
		self.connect(self.canvasTimer, QtCore.SIGNAL("timeout()"), self.graphUpdate)
		self.connect(self.canvasFastTimer, QtCore.SIGNAL("timeout()"), self.graphFastEnds)
		if dict['dock'] == 'true':
			self.changeDock()

		# Variables needed to switch the state of components when double-clicking over them.
		self.clickTimes = 0
		self.lastClickTime = QtCore.QTime()
		self.clickNumber = -1

		# Component config containter
		self.compConfig = []

		# Init component sets
		self.back_comps = set()
		self.requests = set()

		# Icon and system's tray stuff
		self.iconOK = QtGui.QIcon(QtGui.QPixmap('/opt/robocomp/share/rcmanager/drawing_red.png'))
		self.iconFULL = QtGui.QIcon(QtGui.QPixmap('/opt/robocomp/share/rcmanager/drawing_green.png'))
		self.iconChange1 = QtGui.QIcon(QtGui.QPixmap('/opt/robocomp/share/rcmanager/drawing_right.png'))
		self.iconChange2 = QtGui.QIcon(QtGui.QPixmap('/opt/robocomp/share/rcmanager/drawing_left.png'))
		self.setWindowIcon(self.iconOK)
		self.state = 0
		self.doExit = False
		self.systray = None
		self.blinkTimer = QtCore.QTimer()
		self.doDock = False

		# Set the fixed timeout for component checking
		self.timer = QtCore.QTimer()
		self.timer.start(dict['fixed'])


		self.menu = QtGui.QMenuBar(None)
		self.ui.verticalLayout_3.insertWidget(0, self.menu)
		self.menuFile = self.menu.addMenu('File')
		self.menuSim = self.menu.addMenu('Simulation')
		self.menuActions = self.menu.addMenu('Actions')

		self.actionKillAll = self.menuActions.addAction('kill all')
		self.connect(self.actionKillAll, QtCore.SIGNAL("triggered(bool)"), self.killall)
		#self.actionRunAll = self.menuActions.addAction('run all')
		#self.connect(self.actionRunAll, QtCore.SIGNAL("triggered(bool)"), self.runall)

		self.actionOpen = self.menuFile.addAction('Open')
		self.connect(self.actionOpen, QtCore.SIGNAL("triggered(bool)"), self.openFile)
		self.actionSave = self.menuFile.addAction('Save')
		self.connect(self.actionSave, QtCore.SIGNAL("triggered(bool)"), self.saveFile)
		self.actionEdit = self.menuFile.addAction('Edit')
		self.connect(self.actionEdit, QtCore.SIGNAL("triggered(bool)"), self.runEditor)
		self.actionDock = self.menuFile.addAction('Dock')
		self.connect(self.actionDock, QtCore.SIGNAL("triggered(bool)"), self.changeDock)
		self.actionExit = self.menuFile.addAction('Exit')
		self.connect(self.actionExit, QtCore.SIGNAL("triggered(bool)"), self.forceExit)

		# Do we want the eye-candy graph simulation?
		if (dict['active'] == "true"):
			self.doSimulation = True
		else:
			self.doSimulation = False
		if self.doSimulation == True:
			self.actionSS = self.menuSim.addAction('Stop')
		else:
			self.actionSS = self.menuSim.addAction('Start')

		# Set connections
		self.connect(self.ui.checkList, QtCore.SIGNAL("itemClicked(QListWidgetItem *)"), self.selectCheck)
		self.connect(self.ui.upButton, QtCore.SIGNAL("clicked()"), self.up)
		self.connect(self.ui.downButton, QtCore.SIGNAL("clicked()"), self.down)
		self.connect(self.ui.tabWidget, QtCore.SIGNAL("currentChanged(int)"), self.tabChanged)
		self.connect(self.timer, QtCore.SIGNAL("timeout()"), self.checkAll)
		self.connect(self.canvas, QtCore.SIGNAL('upRequest()'), self.manageGraphUp)
		self.connect(self.canvas, QtCore.SIGNAL('downRequest()'), self.manageGraphDown)
		self.connect(self.canvas, QtCore.SIGNAL('configRequest()'), self.manageGraphConfig)
		self.connect(self.actionSS, QtCore.SIGNAL("triggered(bool)"), self.sSimulation)

		# Draw the graph
		self.canvas.update()

		# Get settings
		settings = QtCore.QSettings("RoboComp", "rcmanager")
		value = settings.value("geometry").toByteArray()
		if value != None:
			self.restoreGeometry(value)
		value = settings.value("page").toInt()
		if value != None:
			if value[1] == True:
				self.ui.tabWidget.setCurrentIndex(value[0])
		value = settings.value("docking").toBool()
		if value != None:
			if value == True:
				self.changeDock()

	# Select a new rcmanager configuration file
	def openFile(self, p=None):
		if p==None:
			self.configFile = QtGui.QFileDialog.getOpenFileName (self, "Select file", initDir, "*.xml")
		else:
			self.configFile = p

		if len(self.configFile) > 0:
			if self.canvas.ui != None: self.canvas.ui.close()
			self.readConfig()
		else:
			print 'len(cfgFile) == 0'


	# Save the current configuration to a new file
	def saveFile(self):
		global dict
		if self.canvas.ui != None: self.canvas.ui.close()
		s = QtGui.QFileDialog.getSaveFileName (self, "Select output file", os.environ['HOME'], "*.xml")
		if len(s) > 0:
			for c1 in self.compConfig:
				for c2 in self.canvas.compList:
					if c1.alias == c2.name:
						c1.x = c2.x
						c1.y = c2.y
						c1.r = c2.r
			rcmanagerConfig.writeConfigToFile(dict, self.compConfig, s)

	# Dock icon blinking method.
	def changeDock(self):
		global dict
		if self.canvas.ui != None: self.canvas.ui.close()
		if self.doDock == False:
			self.systray = QtGui.QSystemTrayIcon(self)
			self.systray.setIcon(self.iconOK)
			self.systray.setVisible(True)
			self.connect(self.systray, QtCore.SIGNAL("activated (QSystemTrayIcon::ActivationReason)"), self.toggle)
			self.connect(self.blinkTimer, QtCore.SIGNAL("timeout()"), self.changeIcon)
			self.iconNumber = 0
			self.doDock = True
			dict['dock'] = 'true'
			self.actionDock.setText('Undock')
		else:
			self.systray.deleteLater()
			self.disconnect(self.blinkTimer, QtCore.SIGNAL("timeout()"), self.changeIcon)
			self.iconNumber = 0
			self.doDock = False
			dict['dock'] = 'false'
			self.actionDock.setText('Dock')

	# Stop graph simulation if its running or vice versa.
	def sSimulation(self):
		global dict
		self.doSimulation = not self.doSimulation
		if self.doSimulation == False:
			self.actionSS.setText('Start')
			if self.fastState == False:
				self.canvasTimer.start(dict['focustime'])
			dict['active'] = 'false'
		else:
			self.actionSS.setText('Stop')
			self.setFastState()
			if self.fastState == False:
				self.canvasTimer.start(dict['idletime'])
			dict['active'] = 'true'

	# When doing simulation calling this method will make the simulation go fast
	def setFastState(self, fast=True):
		global dict
		self.fastState = fast
		if fast:
			self.canvasTimer.start(dict['fasttime'])
			self.canvasFastTimer.start(dict['fastperiod'])
		else:
			self.canvasFastTimer.stop()
			if self.ui.tabWidget.currentIndex() == 1 and self.doSimulation == True:
				self.canvasTimer.start(dict['focustime'])
			else:
				self.canvasTimer.start(dict['idletime'])

	# Opposite of the previous method
	def graphFastEnds(self):
		self.setFastState(False)

	# Run component simulator
	def runEditor(self):
		if self.canvas.ui != None: self.canvas.ui.close()
		self.editor = rcmanagerEditor.rcmanagerEditorWidget()
		self.editor.setModal(True)
		self.editor.show()
		self.editor.readConfig(self.configFile)
		self.connect(self.editor, QtCore.SIGNAL('finished()'), self.readConfig)

	# Add the ui-selected component to the requests set by calling the 'up()' method.
	def manageGraphUp(self):
		for idx in range(self.ui.checkList.count()):
			if self.ui.checkList.item(idx).text() == self.canvas.request:
				self.ui.checkList.setCurrentRow(idx)
				self.selectCheck()
				self.up()
				break

	# Add the ui-selected component to the down set by calling the 'down()' method.
	def manageGraphDown(self):
		for idx in range(self.ui.checkList.count()):
			if self.ui.checkList.item(idx).text() == self.canvas.request:
				self.ui.checkList.setCurrentRow(idx)
				self.selectCheck()
				self.down()
				break

	# Edit a component's configuration
	def manageGraphConfig(self):
		for idx in range(self.ui.checkList.count()):
			if self.ui.checkList.item(idx).text() == self.canvas.request:
				self.ui.checkList.setCurrentRow(idx)
				self.selectCheck()
				self.config()
				break

	# Update the UI graph
	def graphUpdate(self):
		global dict
		self.canvas.checkForNewComponents(self)

		self.canvas.center()
		if self.doSimulation:
			self.canvas.step(self)
		self.canvas.update()

	# Current tab changed
	@QtCore.pyqtSignature("int")
	def tabChanged(self, num):
		if self.fastState == False:
			if num == 0: self.canvasTimer.start(dict['idletime'])
			elif num == 1 and self.doSimulation == True: self.canvasTimer.start(dict['focustime'])

	# Retuns True if the specified component is up, otherwise returns False
	def itsUp(self, compNumber):
		if self.compConfig[compNumber].alias in self.componentChecker:
			return self.componentChecker[self.compConfig[compNumber]].isalive()
		return False

	# Queues the user's request to change the state of a given component, turning it off if it's on and viceversa.
	def switchComponent(self, compNumber):
		if self.itsUp(compNumber) == True: self.down()
		else: self.up()

	# Queues the user request to turn on a component
	def up(self):
		itsconfig = self.compConfig[self.ui.checkList.currentRow()]
		self.requests = self.requests | set([itsconfig.alias])
		self.clearFocus()

	# Queues the user request to turn off a component
	def down(self):
		self.bg_exec(str(self.ui.downEdit.text()), self.ui.wdEdit.text())
		self.clearFocus()

	def killall(self):
		for info in self.compConfig:
			self.bg_exec(str(info.compdown), str(info.workingdir))

	# Run the configured file editor
	def config(self):
		global dict
		self.bg_exec(self.compConfig[self.ui.checkList.currentRow()].configFile, self.ui.wdEdit.text())
		self.clearFocus()

	# Reads new configuration from file
	def readConfig(self):
		self.canvas.initialize()
		self.ui.checkList.clear()
		self.compConfig = []
		self.back_comps = set()
		self.requests = set()

		newList, newDict = rcmanagerConfig.getConfigFromFile(self.configFile)

		for k, v in newDict.iteritems():
			dict[k] = v

		self.componentChecker.clear()
		for listItem in newList:
			item = QtGui.QListWidgetItem()
			item.setText(listItem.alias)
			self.ui.checkList.insertItem(0, item)
			self.compConfig.insert(0, listItem)
			self.componentChecker[listItem.alias] = ComponentChecker(listItem.endpoint)
			self.componentChecker[listItem.alias].runrun()

		self.log('Configuration loaded')

		n = rcmanagerConfig.unconnectedGroups(newList)
		if n > 1:
			msg = 'WARNING: ' + str(n) + ' unconnected component groups'
			self.log(msg)
			QtGui.QMessageBox.warning(self, 'Warning', msg)
		self.setFastState()

		# Call-back when



	#
	def selectCheck(self):
		# Check if it's a consecutive click
		notTheLastOneAtTime = 0
		if self.clickNumber != self.ui.checkList.currentRow():
			notTheLastOneAtTime = 1
		if self.lastClickTime.elapsed() > dict['interval']:
			notTheLastOneAtTime = 1
		if notTheLastOneAtTime == 0:                   # It's not
			self.clickTimes = self.clickTimes + 1
		else:                                          # It is
			self.clickTimes = 1
		self.clickNumber = self.ui.checkList.currentRow()
		self.lastClickTime = self.lastClickTime.currentTime()
		# If it's a N-ary click: swap its state
		if self.clickTimes >= dict['switch']:
			self.switchComponent(self.clickNumber)
		# Show information of the last clicked component
		info = self.compConfig[self.ui.checkList.currentRow()]
		self.ui.checkEdit.setText(info.endpoint)
		self.ui.wdEdit.setText(info.workingdir)
		self.ui.upEdit.setText(info.compup)
		self.ui.downEdit.setText(info.compdown)
		self.ui.cfgEdit.setText(info.configFile)

	def checkAll(self, initial=False):
		allOk = True
		workingComponents = set()
		for numItem in range(0, len(self.compConfig)):
			ok = True
			itemConfig = self.compConfig[numItem]
			item = self.ui.checkList.item(numItem)
			if (itemConfig.alias in self.componentChecker) and (self.componentChecker[itemConfig.alias].isalive()):
				item.setTextColor(QtGui.QColor(0, 255, 0))
				workingComponents.add(itemConfig.alias)
			else:
				item.setTextColor(QtGui.QColor(255, 0, 0))
				allOk = False

		if workingComponents != self.back_comps:
			if allOk == False:
				self.blinkTimer.stop()
				self.blinkTimer.start(dict['blink'])

		for comp in workingComponents.difference(self.back_comps):
			self.log('Now \"' + comp + '\" is up.')
		for comp in self.back_comps.difference(workingComponents):
			self.log('Now \"' + comp + '\" is down.')

		if self.wantsDocking():
			if allOk and len(self.compConfig) > 0:
				self.systray.setIcon(self.iconFULL)
			elif workingComponents != self.back_comps:
				self.systray.setIcon(self.iconOK)

		self.back_comps = workingComponents.copy()
		self.upRequests()

	def upRequests(self):
		future_requests = self.requests
		for alias in self.requests:
			itsconfig = self.getConfigByAlias(alias)
			unavailableDependences = []
			for dep in itsconfig.dependences:
				if (not dep in self.componentChecker) or (not self.componentChecker[dep].isalive()):
					unavailableDependences.append(dep)
			if len(unavailableDependences) == 0:
				print 'rcmanager:', alias, 'is now ready to run.'
				self.upConfig(itsconfig)
				future_requests = future_requests - set([alias])
			else:
				print 'rcmanager:', alias, 'has unavailable dependences:', unavailableDependences
				future_requests = future_requests | set(unavailableDependences)
		self.requests = future_requests


	# Tries to execute a component
	def upConfig(self, conf):
		self.bg_exec(conf.compup, conf.workingdir)


	# Executes a command in the background
	def bg_exec(self, command, workDir):
		# Get command argument list
		argument_list = command.split(' ')
		# Set program as argument_list[0]
		program = argument_list[0]
		# Set args as argument_list[1, -1]
		args = argument_list[1:]

		currentWorkDir = os.getcwd()
		os.chdir(workDir)
		proc = QtCore.QProcess()
		print '\nQProcess::startDetached( ' + program + ' , ' + str(args) + ' ) @ ' + os.getcwd() + '\n'
		proc.startDetached(program, args)
		os.chdir(currentWorkDir)

	#
	# Changes the icon of the program properly, skipping if docking is not active
	def changeIcon(self):
		if self.isActiveWindow() == True:
			self.blinkTimer.stop()
			self.systray.setIcon(self.iconOK)
		else:
			if self.iconNumber == 0:
				self.systray.setIcon(self.iconChange1)
				self.iconNumber = 1
			elif self.iconNumber == 1:
				self.systray.setIcon(self.iconChange2)
				self.iconNumber = 2
			else:
				self.systray.setIcon(self.iconChange1)
				self.iconNumber = 1
	#
	# (Un)hide the main window
	def toggle(self):
		if self.isVisible(): self.hide()
		else: self.show()
	#
	# Manages close events
	def closeEvent(self, closeevent):
		settings = QtCore.QSettings("RoboComp", "rcmanager");
		g = self.saveGeometry()
		settings.setValue("geometry", QtCore.QVariant(g))
		settings.setValue("page", QtCore.QVariant(self.ui.tabWidget.currentIndex()))
		settings.setValue("docking", QtCore.QVariant(self.wantsDocking()))
		if self.doExit != 1 and self.doDock == True:
			closeevent.ignore()
			self.hide()
		elif self.wantsDocking():
			closeevent.accept()
			for key, checker in self.componentChecker.iteritems():
				checker.stop()
#		else:
#			closeevent.accept()
#			self.forceExit()
#			sys.exit(0)


	#
	# Forces the program to exit
	def forceExit(self):
		self.doExit = 1
		self.close()
	#
	# Clears the interface selection when the user presses 'Esc'
	def keyPressEvent(self, keyevent):
		if keyevent.key() == 16777216:#0x01000000
			self.ui.checkList.clearSelection()
			if self.canvas.ui != None: self.canvas.ui.close()
	#
	# Interface stuff:
	def uiChange(self):
		self.ui.checkList.setCurrentRow(0)
		self.selectCheck()
		self.clearFocus()
	def clearFocus(self):
		self.ui.checkList.clearSelection()
	def log(self, text):
		self.ui.outputText.append(' * ' + QtCore.QTime.currentTime().toString() + ': ' + text)
	def getConfigByAlias(self, alias):
		for config in self.compConfig:
			if config.alias == alias:
				return config
		return None
	#
	# Return 1 if docking is selected, 0 otherwise.
	def wantsDocking(self):
		if self.doDock == True: return 1
		else: return 0
	def resizeEvent(self, e):
		old = e.oldSize()
		new = e.size()
		inc = new - old
		if (inc.width != 0 or inc.height!=0):
			self.canvas.resize(self.canvas.size()+inc)
		e.accept()


class GraphNode:
	def __init__(self):
		self.name = ''
		self.color = None
		self.htmlcolor = None
		self.deps = []
		self.on = False
		self.x = 0.
		self.y = 0.
		self.r = 10.
		self.vel_x = 0.
		self.vel_y = 0.


class GraphView(QtGui.QWidget):
	def __init__(self, parent=None):
		QtGui.QWidget.__init__(self, parent)
		self.tab = parent
		self.initialize()
	def initialize(self):
		global dict
		self.compList = []

		self.VisualNodeCogia = None
		self.ox = 0
		self.oy = 0
		self.ui = None

		#self.hookes_constant = dict['hookes']
		self.spring_length = dict['springlength']
		self.roza = 1.-dict['friction']
		self.time_elapsed2 = dict['step']**2
		self.field_force_multiplier = dict['fieldforce']
		self.hookes_constant = dict['hookes']
		
	def nodes(self):
		if self.VisualNodeCogia:
			return self.compList + list(self.VisualNodeCogia)
		else:
			return self.compList
	def checkForNewComponents(self, parent):
		# Check for components added to the configuration
		anyone = False
		for parentComp in parent.compConfig:
			notFound = True
			if self.VisualNodeCogia:
				if self.VisualNodeCogia.name == parentComp.alias:
					if self.VisualNodeCogia.name in parent.componentChecker:
						notFound = False
						self.VisualNodeCogia.on = parent.componentChecker[self.VisualNodeCogia.name].isalive()
					break
			if notFound:
				for myComp in self.compList:
					if myComp.name == parentComp.alias:
						notFound = False
						myComp.on = parent.componentChecker[myComp.name].isalive()
						break
			if notFound:
				newOne = GraphNode()
				newOne.color = parentComp.color
				newOne.htmlcolor = parentComp.htmlcolor
				newOne.name = parentComp.alias
				newOne.deps = parentComp.dependences
				newOne.x = float(parentComp.x)
				newOne.y = float(parentComp.y)
				newOne.r = float(parentComp.r)
				self.compList.append(newOne)
				anyone = True
		#if anyone == True: self.step(self)
	def step(self, parent):
		#
		# Compute velocities
		for iterr in self.compList:
			force_x = force_y = 0.
			for iterr2 in self.compList:
				if iterr.name == iterr2.name: continue
				ix = iterr.x - iterr2.x
				iy = iterr.y - iterr2.y
				while ix == 0 and iy == 0:
					iterr.x = iterr.x + random.uniform(  -10, 10)
					iterr2.x = iterr2.x + random.uniform(-10, 10)
					iterr.y = iterr.y + random.uniform(  -10, 10)
					iterr2.y = iterr2.y + random.uniform(-10, 10)
					ix = iterr.x - iterr2.x
					iy = iterr.y - iterr2.y

				angle = math.atan2(iy, ix)
				dist2 = ((abs((iy*iy) + (ix*ix))) ** 0.5) ** 2.
				if dist2 < self.spring_length: dist2 = self.spring_length
				force = self.field_force_multiplier / dist2
				force_x += force * math.cos(angle)
				force_y += force * math.sin(angle)

			for iterr2 in self.compList:
				if iterr2.name in iterr.deps or iterr.name in iterr2.deps:
					ix = iterr.x - iterr2.x
					iy = iterr.y - iterr2.y
					angle = math.atan2(iy, ix)
					force = math.sqrt(abs((iy*iy) + (ix*ix)))      # force means distance actually
					#if force <= self.spring_length: continue       # "
					force -= self.spring_length                    # force means spring strain now
					force = force * self.hookes_constant           # now force means force :-)
					force_x -= force*math.cos(angle)
					force_y -= force*math.sin(angle)

			iterr.vel_x = (iterr.vel_x + (force_x*self.time_elapsed2))*self.roza
			iterr.vel_y = (iterr.vel_y + (force_y*self.time_elapsed2))*self.roza

		# Update positions
		for iterr in self.compList:
			iterr.x += iterr.vel_x
			iterr.y += iterr.vel_y

	def center(self):
		total = 0
		totalx = 0.
		totaly = 0.
		for iterr in self.compList:
			totalx += iterr.x
			totaly += iterr.y
			total  += 1
		if self.VisualNodeCogia:
			totalx += self.VisualNodeCogia.x
			totaly += self.VisualNodeCogia.y
			total  += 1

		if abs(totalx) > 0.001:
			meanx = totalx / total
			for iterr in self.compList:
				iterr.x -= meanx
			if self.VisualNodeCogia:
				self.VisualNodeCogia.x -= meanx
		if abs(totaly) > 0.001:
			meany = totaly / total
			for iterr in self.compList:
				iterr.y -= meany
			if self.VisualNodeCogia:
				self.VisualNodeCogia.y -= meany

	def paintNode(self, node):
		w2 = self.parent().width()/2
		h2 = self.parent().height()/2+30
		global dict

		if node.on:
			self.painter.setBrush(QtGui.QColor(0, 255, 0, dict['alpha']))
			self.painter.setPen(QtGui.QColor(0, 255, 0))
		else:
			self.painter.setBrush(QtGui.QColor(255, 0, 0, dict['alpha']))
			self.painter.setPen(QtGui.QColor(255, 0, 0))
		self.painter.drawEllipse(node.x-node.r+w2, node.y-node.r+h2, node.r*2, node.r*2)

		self.painter.drawText(QtCore.QPoint(node.x-node.r+w2, node.y-node.r-3+h2), node.name)

		if node.color != None:
			self.painter.setBrush(node.color)
			self.painter.setPen(node.color)
			self.painter.drawEllipse(node.x-node.r/4+w2, node.y-node.r/4+h2, node.r/2, node.r/2)


	def paintEvent(self, event):	
		w2 = self.tab.width()/2
		h2 = self.tab.height()/2+30
		nodosAPintar = [] + self.compList
		if self.VisualNodeCogia: nodosAPintar.append(self.VisualNodeCogia)

		self.painter = QtGui.QPainter(self)
		self.painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

		for i in nodosAPintar:
			xo = i.x
			yo = i.y
			for j in  nodosAPintar:
				if j.name in i.deps:
					angle = 180.-(math.atan2(yo-j.y, xo-j.x)*(57.2957795))
					xinc = j.x - i.x
					yinc = j.y - i.y

					mag =  ( xinc**2. + yinc**2. )**0.5
					if mag == 0: continue


					xshift = (xinc/mag)
					yshift = (yinc/mag)

					xinit = i.x+xshift*i.r
					yinit = i.y+yshift*i.r
					xend = xinit+((mag-i.r-j.r)*math.cos(angle*(math.pi/180.)))
					yend = yinit-((mag-i.r-j.r)*math.sin(angle*(math.pi/180.)))

					self.painter.setPen(QtGui.QColor(0, 0, 255, 150))
					self.painter.drawLine(xinit+w2, yinit+h2, xend+w2, yend+h2)
					self.painter.setBrush(QtGui.QColor(0, 0, 255, 200))
					#-j.r-(xshift/i.r)*j.r,
					#-j.r-(yshift/i.r)*j.r
					px = j.x-10-xshift*j.r
					py = j.y-10-yshift*j.r
					self.painter.drawPie(px+w2, py+h2, 20, 20, abs((angle+180-16)*16), 32*16)

		self.painter.setFont(QtGui.QFont("Arial", 13));
		for i in self.compList:
			self.paintNode(i)
		if self.VisualNodeCogia:
			self.paintNode(self.VisualNodeCogia)
		self.painter = None

	def mousePressEvent(self, e):
		self.showNodeMenu(e)
	def mouseDoubleClickEvent(self, e):
		self.showNodeMenu(e, True)
	def showNodeMenu(self, e, forceDialog=False):
		w2 = self.parent().width()/2
		h2 = self.parent().height()/2 + 30
		x = e.x()-w2
		y = e.y()-h2
		if self.ui: self.ui.close()
		VisualNode = None
		minDist = -1.
		minIndex = 0
		for b in self.compList:
			bx = b.x
			by = b.y
			dist = (  (bx-x)**2  +  (by-y)**2  )**0.5
			if dist < b.r:
				if dist < minDist or minDist == -1.:
					VisualNode = b
					minDist = dist
					minIndex = self.compList.index(b)
					self.ox = x - b.x
					self.oy = y - b.y
		if VisualNode:
			if e.button() == 2 or forceDialog:
				self.ui = CommandDialog(self, self.compList[minIndex].x+w2, self.compList[minIndex].y+h2)
				self.ui.idx = minIndex
				self.connect(self.ui, QtCore.SIGNAL('up()'), self.up)
				self.connect(self.ui, QtCore.SIGNAL('down()'), self.down)
				self.connect(self.ui, QtCore.SIGNAL('config()'), self.config)
				self.ui.show()
			elif e.button() == 1:
				self.VisualNodeCogia = self.compList.pop(minIndex)
		self.repaint()
	def mouseReleaseEvent(self, e):
		if self.VisualNodeCogia != None:
			self.compList.append(self.VisualNodeCogia)
			self.VisualNodeCogia = None
			self.emit(QtCore.SIGNAL("nodeReleased()"))
	def mouseMoveEvent(self, e):
		w2 = self.parent().width()/2
		h2 = self.parent().height()/2+30
		self.repaint()
		if self.VisualNodeCogia != None:
			self.VisualNodeCogia.x = e.x()-self.ox-w2
			self.VisualNodeCogia.y = e.y()-self.oy-h2
			self.repaint()
	def wheelEvent(self, e):
   		if e.delta() > 0:
   			for i in self.compList:
   				i.x = i.x * 1.1
   				i.y = i.y * 1.1
   				i.r = i.r * 1.1
   			self.repaint
   		else:
   			for i in self.compList:
   				i.x = i.x * 0.9
   				i.y = i.y * 0.9
   				i.r = i.r * 0.9
   			self.repaint
	def up(self):
		self.request = self.compList[self.ui.idx].name
		self.ui.close()
		self.emit(QtCore.SIGNAL("upRequest()"))
	def down(self):
		self.request = self.compList[self.ui.idx].name
		self.ui.close()
		self.emit(QtCore.SIGNAL("downRequest()"))
	def config(self):
		self.request = self.compList[self.ui.idx].name
		self.emit(QtCore.SIGNAL("configRequest()"))
		self.ui.close()



#
# Create the Qt application, the class, and runs the program
#
if __name__ == '__main__':
	app = QtGui.QApplication(sys.argv)
	window = TheThing()
	
	if len(sys.argv) > 1:
		if os.path.isfile(sys.argv[1]):
			window.show()
			window.openFile(sys.argv[1])

			ret = -1

			try:
				ret = app.exec_()
			except:
				print 'Some error happened.'
		else:
			print sys.argv[1] + " does not exist"	
	else: 
		print "Please enter an argument. Ex - rcmanager sample.xml"

	sys.exit()
