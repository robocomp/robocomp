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

class MainClass(QtGui.QMainWindow):
	"""docstring for MainClass"""
	def __init__(self, arg=None):
		QtGui.QDialog.__init__(self,arg)
		self.currentComponent=None
		self.setWindowIcon(QtGui.QIcon(QtGui.QPixmap("/opt/robocomp/share/rcmanager/drawing_green.png")))
		self.showMaximized()
		self.componentList=[]
		self.networkSettings=rcmanagerConfig.NetworkValues()
		

		self.UI=ui_formManager.Ui_MainWindow()
		self.UI.setupUi(self)
		self.UI.tabWidget.removeTab(0)
		self.Logger=rcmanagerConfig.Logger(self.UI.textBrowser)

		self.SaveWarning=rcmanagerConfig.SaveWarningDialog(self)
		
		self.NetworkScene=rcmanagerConfig.ComponentScene(self)##The graphicsScene
		self.graphTree = rcmanagerConfig.ComponentTree(self.UI.frame,mainclass=self)##The graphicsNode
		self.graphTree.setScene(self.NetworkScene)
		self.graphTree.setObjectName(_fromUtf8("graphicsView"))
		self.UI.gridLayout_8.addWidget(self.graphTree,0,0,1,1)
		self.setZoom()
		
		#This will read the the network setting from xml files and will set the values
		self.networkSettingDialog=rcmanagerConfig.NetworkSettings(self)
		#tool always works either on opened xml file or user dynamically build xml file.
		#So the two variable given below will always be the negation of each other
		self.FileOpenStatus=False
		self.UserBuiltNetworkStatus=True
		
		#To track the changes in the network both functionaly and visually
		self.HadChanged=False
		
		self.LogFileSetter=rcmanagerConfig.LogFileSetter(self,self.Logger)
		self.simulatorTimer=QtCore.QTimer()

		self.connectionBuilder=rcmanagerConfig.connectionBuilder(self,self.Logger)##This will take care of connection building between components

		self.PositionMultiplier=rcmanagerConfig.PositionMultiplier(self.Logger)

		self.groupBuilder=rcmanagerConfig.GroupBuilder(self,self.Logger)##It will help to create a new group
		#setting the code Editor
		
		self.groupSelector=rcmanagerConfig.GroupSelector(self,self.Logger)

		self.CodeEditor=rcmanagerConfig.CodeEditor(self.UI.tab_2)
		self.UI.verticalLayout_2.addWidget(self.CodeEditor)

		#The small widget which appears when we right click on a node in tree
		
		self.nodeDetailDisplayer=rcmanagerConfig.ShowItemDetails(self.graphTree)

		#Setting up the connection
		self.setupActions()

		#self.textEdit=QtGui.QTextEdit()#Temp
		#self.UI.tabWidget_2.addTab(self.textEdit,"Helllo")#Temp
		#print "Count is "+ str(self.UI.verticalLayout.count())
		#self.UI.toolButton_6.setMouseTracking(True)
		
	def setupActions(self):##To setUp connection like saving,opening,etc
		self.connect(self.simulatorTimer,QtCore.SIGNAL("timeout()"),self.simulate)
		#self.connect(self.UI.toolButton,QtCore.SIGNAL("hovered()"),self.hoverAddComponent)
		#self.connect(self.UI.toolButton_9,QtCore.SIGNAL("hovered()"),self.hoverXmlSettings)
		#self.connect(self.UI.toolButton_5,QtCore.SIGNAL("hovered()"),self.hoverPrintDefaultNode)
		#self.connect(self.UI.toolButton_4,QtCore.SIGNAL("hovered()"),self.hoverPrintDefaultSettings)
		#self.connect(self.UI.toolButton_3,QtCore.SIGNAL("hovered()"),self.hoverRefreshFromXml)
		#self.connect(self.UI.toolButton_10,QtCore.SIGNAL("hovered()"),self.hoverNetworkTreeSettings)
		#self.connect(self.UI.toolButton_6,QtCore.SIGNAL("hovered()"),self.hoverRefreshFromTree)
		self.connect(self.UI.actionSet_Log_File,QtCore.SIGNAL("triggered(bool)"),self.setLogFile)

		self.connect(self.UI.tabWidget,QtCore.SIGNAL("currentChanged(int)"),self.tabIndexChanged)
		self.connect(self.UI.actionSave,QtCore.SIGNAL("triggered(bool)"),self.saveXmlFile)
		self.connect(self.UI.actionOpen,QtCore.SIGNAL("triggered(bool)"),self.openXmlFile)
		self.connect(self.UI.actionExit,QtCore.SIGNAL("triggered(bool)"),self.exitRcmanager)
		self.connect(self.UI.actionSetting,QtCore.SIGNAL("triggered(bool)"),self.rcmanagerSetting)
		self.connect(self.UI.actionON,QtCore.SIGNAL("triggered(bool)"),self.simulatorOn)
		self.connect(self.UI.actionOFF,QtCore.SIGNAL("triggered(bool)"),self.simulatorOff)
		self.connect(self.UI.actionSetting_2,QtCore.SIGNAL("triggered(bool)"),self.simulatorSettings)
		self.connect(self.UI.actionSetting_3,QtCore.SIGNAL("triggered(bool)"),self.controlPanelSettings)
		self.connect(self.UI.actionSetting_4,QtCore.SIGNAL("triggered(bool)"),self.editorSettings)
		self.connect(self.graphTree.BackPopUpMenu.ActionUp,QtCore.SIGNAL("triggered(bool)"),self.upAllComponents)
		self.connect(self.graphTree.BackPopUpMenu.ActionDown,QtCore.SIGNAL("triggered(bool)"),self.downAllComponents)
		self.connect(self.graphTree.BackPopUpMenu.ActionSearch,QtCore.SIGNAL("triggered(bool)"),self.searchInsideTree)
		self.connect(self.graphTree.BackPopUpMenu.ActionAdd,QtCore.SIGNAL("triggered(bool)"),self.addNewNode)		
		self.connect(self.graphTree.BackPopUpMenu.ActionSettings,QtCore.SIGNAL("triggered(bool)"),self.setNetworkSettings)
		self.connect(self.graphTree.BackPopUpMenu.ActionNewGroup,QtCore.SIGNAL("triggered(bool)"),self.addNewGroup)
		self.connect(self.graphTree.BackPopUpMenu.ActionStretch,QtCore.SIGNAL("triggered(bool)"),self.stretchGraph)

		self.connect(self.graphTree.CompoPopUpMenu.ActionEdit,QtCore.SIGNAL("triggered(bool)"),self.EditSelectedComponent)
		self.connect(self.graphTree.CompoPopUpMenu.ActionDelete,QtCore.SIGNAL("triggered(bool)"),self.deleteSelectedComponent)
		self.connect(self.graphTree.CompoPopUpMenu.ActionUp,QtCore.SIGNAL("triggered(bool)"),self.upSelectedComponent)
		self.connect(self.graphTree.CompoPopUpMenu.ActionAddToGroup,QtCore.SIGNAL("triggered(bool)"),self.addComponentToGroup)
		self.connect(self.graphTree.CompoPopUpMenu.ActionDown,QtCore.SIGNAL("triggered(bool)"),self.downSelectedComponent)
		self.connect(self.graphTree.CompoPopUpMenu.ActionNewConnection,QtCore.SIGNAL("triggered(bool)"),self.BuildNewConnection)
		self.connect(self.graphTree.CompoPopUpMenu.ActionControl,QtCore.SIGNAL("triggered(bool)"),self.controlComponent)
		self.connect(self.graphTree.CompoPopUpMenu.ActionRemoveFromGroup,QtCore.SIGNAL("triggered(bool)"),self.componentRemoveFromGroup)
		self.connect(self.graphTree.CompoPopUpMenu.ActionUpGroup,QtCore.SIGNAL("triggered(bool)"),self.upGroup)
		self.connect(self.graphTree.CompoPopUpMenu.ActionDownGroup,QtCore.SIGNAL("triggered(bool)"),self.downGroup)
		#self.connect(self.graphTree.CompoPopUpMenu.ActionFreq,QtCore.SIGNAL("triggered(bool)"),self.getFreq)

		self.connect(self.UI.toolButton_2,QtCore.SIGNAL("clicked()"),self.searchEnteredAlias)
		self.connect(self.UI.toolButton_7,QtCore.SIGNAL("clicked()"),self.simulatorOn)
		self.connect(self.UI.toolButton_8,QtCore.SIGNAL("clicked()"),self.simulatorOff)

		self.connect(self.SaveWarning,QtCore.SIGNAL("save()"),self.saveXmlFile)
		self.connect(self.UI.toolButton_3,QtCore.SIGNAL("clicked()"),self.refreshTreeFromCode)
		self.connect(self.UI.toolButton_4,QtCore.SIGNAL("clicked()"),self.addNetworkTempl)
		self.connect(self.UI.toolButton_5,QtCore.SIGNAL("clicked()"),self.addComponentTempl)
		self.connect(self.UI.toolButton_6,QtCore.SIGNAL("clicked()"),self.refreshCodeFromTree)
		self.connect(self.UI.toolButton_9,QtCore.SIGNAL("clicked()"),self.editorFontSettings)
		#self.connect(self.UI.toolButton_10,QtCore.SIGNAL("clicked()"),self.getNetworkSetting)(Once finished Uncomment this)
		self.connect(self.UI.toolButton,QtCore.SIGNAL("clicked()"),self.addNewComponent)
		self.Logger.logData("Tool Started")
	def getFreq(self):
		comp=self.graphTree.CompoPopUpMenu.currentComponent.parent 
		comp.CheckItem.getFreq()
	def EditSelectedComponent(self):	
		self.UI.tabWidget.setCurrentIndex(1)
		self.CodeEditor.findFirst(self.graphTree.CompoPopUpMenu.currentComponent.parent.alias,False,True,True,True)

	def setLogFile(self):
		self.LogFileSetter.setFile() 

	def stretchGraph(self):
		self.PositionMultiplier.updateStretch(self.componentList,self.networkSettings)

	def upGroup(self):
		component=self.graphTree.CompoPopUpMenu.currentComponent.parent
		if component.group!=None:
			component.group.upGroupComponents(self.Logger)
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
			self.Logger.logData("No group","R")
	def addComponentToGroup(self):
		component=self.graphTree.CompoPopUpMenu.currentComponent.parent
		self.groupSelector.openSelector(component,self.networkSettings.Groups)
		self.NetworkScene.update()
		self.refreshCodeFromTree()
	def BuildNewConnection(self):

		self.graphTree.connectionBuidingStatus=True
		print "Connection Building"
		self.connectionBuilder.buildNewConnection()
		self.connectionBuilder.setBeg(self.graphTree.CompoPopUpMenu.currentComponent.parent)
		self.connectionBuilder.show()
		

	def addNewGroup(self):
		self.groupBuilder.startBuildGroup(self.networkSettings)

	def deleteSelectedComponent(self):
		self.deleteComponent(self.graphTree.CompoPopUpMenu.currentComponent.parent)

	def tabIndexChanged(self):##This will make sure the common behavior is not working unneccessarily 
		index=self.UI.tabWidget.currentIndex()
		if index==1 or index==2:##CommonProxy should only work if the first tab is visible
			if self.currentComponent != None:
				self.currentComponent.CommonProxy.setVisibility(False)
	def addNetworkTempl(self):
		string=rcmanagerConfig.getDefaultSettings()
		pos=self.CodeEditor.getCursorPosition()
		self.CodeEditor.insertAt(string,pos[0],pos[1])
	def getNetworkSetting(self):#This will show the Network setting Dialog box and will help to update the stuffs
		self.networkSettingDialog.setData(self.networkSettings)
		self.networkSettingDialog.show()
	def editorFontSettings(self):#BUUUUUUUUUUUUUGGGG
		font,ok=QtGui.QFontDialog.getFont()
		if ok:
			self.CodeEditor.setFont(font)
			self.CodeEditor.font=font
			self.Logger.logData("New font set for code Editor")
	def refreshCodeFromTree(self):
		self.HadChanged=True
		string=rcmanagerConfig.getXmlFromNetwork(self.networkSettings,self.componentList,self.Logger)
		self.CodeEditor.setText(string)
		self.Logger.logData("Code Updated SucceFully from the graph")
	def refreshTreeFromCode(self,firstTime=False):#This will refresh the code (Not to file)and draw the new tree
		#print "Refreshing"
		try:
			List,Settings=rcmanagerConfig.getDataFromString(str(self.CodeEditor.text()),self.Logger)
		except Exception,e:
			self.Logger.logData("Error while updating tree from Code::"+str(e), "R")
		else:

			if firstTime==True:
				self.removeAllComponents()
				self.NetworkScene.clear()
				self.networkSettings=Settings
				self.componentList=List
				try :
					#if self.areTheyTooClose()==True:
						#self.theyAreTooClose()
					self.currentComponent=self.componentList[0]
					self.ipCount()
					self.setAllIpColor()
					self.setAllGraphicsData()
					self.drawAllComponents()
					self.setConnectionItems()
					self.drawAllConnection()
					self.setComponentVariables()
					self.setDirectoryItems()
					self.FileOpenStatus=True
					self.UserBuiltNetworkStatus=True
					#self.HadChanged=False
					self.Logger.logData("File Updated SuccessFully from the Code Editor")
					self.refreshCodeFromTree()		
				except Exception,e:
					self.Logger.logData("File updation from Code Failed "+str(e),"R")

			else:
				self.networkSettings=Settings
				for x in List:
					try:
						comp=self.searchforComponent(x.alias)
					except:
						self.componentList.append(x)##This will add a new component


				for x in self.componentList:
					if self.searchInsideList(List,x.alias)==False:
						self.Logger.logData("Deleted the older component ::"+x.alias)##If new tree does have this	
						self.deleteComponent(x)

				for x in self.componentList:
					for y in List:
						if x.alias==y.alias:
							self.copyAndUpdate(x,y)
				self.Logger.logData("Tree Updated succesfully From File")
	def copyAndUpdate(self,original,temp):
		if original.x!=temp.x or original.y!=temp.y:
			original.x=temp.x
			original.y=temp.y
			original.graphicsItem.setPos(original.x,original.y)
			original.graphicsItem.updateforDrag()	
			self.Logger.logData("Position Updated of ::"+original.alias)
		
		original.workingdir=temp.workingdir
		original.compup=temp.compup
		original.compdown=temp.compdown
		original.configFile=temp.configFile
		original.nodeColor=temp.nodeColor

		if original.groupName!=temp.groupName:
			try:
				group=rcmanagerConfig.searchForGroupName(self.networkSettings,original.groupName)
				group.addComponent(original)
			except Exception,e:
				self.Logger.logData(str(e),"R")

		for x in temp.dependences: ##For adding new connection if needed
			if original.dependences.__contains__(x)==False:
				original.dependences.append(x)
				comp=searchforComponent(x)
				self.setAconnection(comp,original)
		for x in original.dependences:##For deleting unwanted connection if needed
			name=x
			if temp.dependences.__contains__(x)==False:
				original.dependences.remove(x)
				for y in original.asEnd:
					if y.fromComponent.alias==name:
						original.asEnd.remove(y)

		if original.Ip!=temp.Ip:
			original.Ip=temp.Ip
			self.ipCount()
			self.setAllIpColor()

		if original.endpoint!=temp.endpoint:
			original.CheckItem.initializeComponent()


	def searchInsideList(self,List,name):
		flag=0
		for x in List:
			if x.alias==name:
				flag+=1
				return True

		if flag==0:
			return False

	def printTemplSettings(self):
		pass
	def addComponentTempl(self):
		string=rcmanagerConfig.getDefaultNode()
		pos=self.CodeEditor.getCursorPosition()
		self.CodeEditor.insertAt(string,pos[0],pos[1])
	def searchEnteredAlias(self):#Called when we type an alias and search it
		try:
			alias=self.UI.lineEdit.text()
			if alias== '':
				raise Exception("No Name Entered")
				
			else :
				x=self.searchforComponent(alias)#Write here what ever should happen then
				self.graphTree.centerOn(x.graphicsItem)
				#x.graphicsItem.setSelected(True)
			self.Logger.logData(alias+"  Found")
		except Exception, e:
			self.Logger.logData("Search Error::  "+ str(e),"R")
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

			if flag==False:
				self.ipList.append(x.Ip)
			
	def setAllIpColor(self):#A small algorithm to allot colors to each Ip
		try:
			diff=int(765/self.ipList.__len__())
			for x in range(self.ipList.__len__()): 
				num=(x+1)*diff
				
				if num<=255:
					for y in self.componentList.__iter__():
						if self.ipList[x]==y.Ip:
							y.graphicsItem.IpColor=QtGui.QColor.fromRgb(num,0,0)
				elif num>255 and num <=510:
					for y in self.componentList.__iter__():
						if self.ipList[x]==y.Ip:
							y.graphicsItem.IpColor=QtGui.QColor.fromRgb(255,num-255,0)
				elif num>510:
					for y in self.componentList.__iter__():
						if self.ipList[x]==y.Ip:
							y.graphicsItem.IpColor=QtGui.QColor.fromRgb(255,255,num-510)
			self.Logger.logData("IpColor Alloted SuccessFully")				
		except Exception,e:
			raise Exception("Error During Alloting Ipcolors "+str(e))
			  	

	def setComponentVariables(self):#Temperory function to be edited later
		for x in self.componentList.__iter__():
			x.View=self.graphTree
			x.mainWindow=self				
			self.connect(x,QtCore.SIGNAL("networkChanged()"),self.haveChanged)

	def setDirectoryItems(self):#This will set and draw all the directory components+I have added the job of defining a connection in here
		for x in self.componentList.__iter__():
			x.DirectoryItem.setParent(self.UI.scrollAreaWidgetContents)
			self.UI.verticalLayout.insertWidget(self.UI.verticalLayout.count()-1,x.DirectoryItem)
		#print "Count is "+ str(self.UI.verticalLayout.count())
	
	def componentSettings(self,component):#To edit the settings of currentComponent
		print "Settings of current component"
	def controlComponent(self,component):#To open up the control panel of current component
		print "Controlling the current component"
	def downSelectedComponent(self):
		component=self.graphTree.CompoPopUpMenu.currentComponent
		rcmanagerConfig.downComponent(component.parent,self.Logger)
	
	

	def upSelectedComponent(self):#This will up a selected component
		component=self.graphTree.CompoPopUpMenu.currentComponent
		rcmanagerConfig.upComponent(component.parent,self.Logger)
	def setNetworkSettings(self):#To edit the network tree general settings
		print "network setting editing"	
	def searchInsideTree(self):#To search a particular component from tree
		print "Searching inside the tree"
	def upAllComponents(self):#To set all components in up position
		for x in self.componentList.__iter__():
			try:
				rcmanagerConfig.upComponent(x,self.Logger)
			except Exception, e:
				pass
	def downAllComponents(self):#To set all components in down position
		for x in self.componentList.__iter__():
			try:
				rcmanagerConfig.downComponent(x,self.Logger)
				time.sleep(.5)
			except Exception,e :
				pass
	def simulatorSettings(self):##To edit the simulatorSettings:Unfinished
		print "Simulator settings is on"
	def controlPanelSettings(self):##To edit the controlPanel Settings:Unfinshed
		print "Control panel settings"
	def editorSettings(self):##To edit the editors settins:Unfinshed
		print "Editor Settings"	
	def simulatorOff(self):	#To switch Off the simulator::Unfiunished
		self.Logger.logData("Simulator Ended")
		self.simulatorTimer.stop()
	def simulatorOn(self):
		self.Logger.logData("Simulator Started")
		self.simulatorTimer.start(300)
	def simulate(self):#To switch ON simulator::Unfinished
		
		for iterr in self.componentList:
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
		
		for iterr in self.componentList:
			#print "updating "+iterr.alias
			iterr.graphicsItem.setPos(QtCore.QPointF(iterr.x,iterr.y))
			iterr.graphicsItem.updateforDrag()

	def simulate2(self):##Another Simulation algorithum Working on that..
		min=self.componentList[0]
		for iterr in self.componentList:
			if iterr.x<min.x:
				min=iterr

				
				
	def rcmanagerSetting(self):#To edit the setting of the entire rcmanager settings tool
		pass
	def exitRcmanager(self):##To exit the tool after doing all required process
		print "Exiting"
	def drawAllComponents(self):#Called to draw the components
		for x in self.componentList.__iter__():
			self.NetworkScene.addItem(x.graphicsItem)

	def drawAllConnection(self):#This will start drawing Item
		for x in self.componentList.__iter__():
			for y in x.asBeg.__iter__():
				self.NetworkScene.addItem(y)

	def setConnectionItems(self):#This is called right after reading from a file,Sets all the connection graphicsItems
		for x in self.componentList.__iter__():
			
			for y in x.dependences.__iter__():
				try :
					comp=self.searchforComponent(y)
					self.setAconnection(comp,x)
					self.Logger.logData("Connection from "+comp.alias+" to "+x.alias+" Set")
				except Exception,e:
					print "Error while setting connection ::"+str(e)

	def searchforComponent(self,alias):#this will search inside the components tree
		flag=False
		for x in self.componentList.__iter__():
			if x.alias==alias:
				flag=True
				return x
		if not flag:
			raise Exception("No such component with alias "+alias)

	
	def setAconnection(self,fromComponent,toComponent):#To set these two components
		
		connection=rcmanagerConfig.NodeConnection()
		
		connection.toComponent=toComponent
		connection.fromComponent=fromComponent
		
		fromComponent.asBeg.append(connection)
		toComponent.asEnd.append(connection)
		
		fromComponentPort,toComponentPort=rcmanagerConfig.findWhichPorts(fromComponent,toComponent)
		
		connection.fromX,connection.fromY=rcmanagerConfig.findPortPosition(fromComponent,fromComponentPort)
		connection.toX,connection.toY=rcmanagerConfig.findPortPosition(toComponent,toComponentPort)	

	def removeAllComponents(self):#This removes all the components from everywhere#BUUUUUG
		len=self.componentList.__len__()
		for x in range(len):
			self.deleteComponent(self.componentList[len-1-x])
	def openXmlFile(self,terminalArg=False,UserHaveChoice=True):#To open the xml files ::Unfinished
		Settings=rcmanagerConfig.NetworkValues()
		List=[]
		try:
			if self.HadChanged :# To make sure the data we have been working on have been saved
				decision=self.SaveWarning.decide()
				if decision=="C":
					raise Exception(" Reason:Canceled by User")
				elif decision=="S":					
					self.saveXmlFile()			
			if terminalArg==False and UserHaveChoice==True:
				self.filePath=QtGui.QFileDialog.getOpenFileName(self,'Open file',initDir,'*.xml')
			
			string=rcmanagerConfig.getStringFromFile(self.filePath)
			self.CodeEditor.setText(string)
		except:
			self.Logger.logData("Couldn't Read from File")	
		self.refreshTreeFromCode(firstTime=True)
		self.HadChanged=False
	def setAllGraphicsData(self):
		for x in self.componentList:
			x.setGraphicsData()
	def StatusBarFileNameWrite(self,string):
		Label=QtGui.QLabel(string)
		self.UI.statusbar.addWidget(Label)
	def saveXmlFile(self):##To save the entire treesetting into a xml file
		try:
			saveFileName=QtGui.QFileDialog.getSaveFileName(self,'Save File',initDir,'*.xml')
			saveFileName=str(saveFileName)
			if saveFileName.endswith(".xml")==False:
				saveFileName=saveFileName+".xml"
			string=self.CodeEditor.text()
			try:
				file = open(saveFileName, 'w')
				file.write(string)
			except:
				raise Exception("Can't Open"+saveFileName)
			#rcmanagerConfig.writeToFile(file,string)

			self.Logger.logData("Saved to File "+saveFileName+" ::SuccessFull")
		except Exception,e:
			self.Logger.logData("Saving to File"+saveFileName+" ::Failed."+str(e),"R")
	
	#def saveTofile(fileName):#Save to this filename
	#	rcmanagerConfig.writeConfigToFile(self.networkSettings,self.componentList,fileName)
			
	
	def setZoom(self): ##To connect the slider motion to zooming
		self.UI.verticalSlider.setRange(-20,20)
		self.UI.verticalSlider.setTickInterval(1)
		self.UI.verticalSlider.setValue(0)
		self.currentZoom=0
		self.UI.verticalSlider.valueChanged.connect(self.graphZoom)
	def graphZoom(self):##To be called when ever we wants to zoomingfactor
		self.graphTree.setTransformationAnchor(self.graphTree.AnchorUnderMouse)
		new=self.UI.verticalSlider.value()
		diff=new-self.currentZoom
		self.currentZoom=new
		zoomingfactor=math.pow(1.2,diff)
		#print zoomingfactor
		self.graphTree.scale(zoomingfactor,zoomingfactor)
	
	def addNewNode(self):##Called where right clicked and seleceted to add a new node
		pos=self.graphTree.BackPopUpMenu.pos
		scenePos=self.graphTree.mapToScene(pos)
		self.addNewComponent(scenePos)	

	def addNewComponent(self,pos=QtCore.QPointF()):#The final function which takes care of adding new component default zero
	
		component=rcmanagerConfig.CompInfo(view=self.graphTree,mainWindow=self,name="Component"+str(self.componentList.__len__()))
		#component.CheckItem.setLogger(self.Logger)
		self.componentList.append(component)
		component.x=pos.x()
		component.y=pos.y()
		component.graphicsItem.setPos(pos)
		self.NetworkScene.addItem(component.graphicsItem)
		self.refreshCodeFromTree()
		component.DirectoryItem.setParent(self.UI.scrollAreaWidgetContents)
		self.UI.verticalLayout.insertWidget(self.UI.verticalLayout.count()-1,component.DirectoryItem)
		self.UI.tabWidget.setCurrentIndex(1)
		self.CodeEditor.findFirst("Component"+str(self.componentList.__len__()-1),False,True,True,True)
	def deleteComponent(self,component):##This will delete the component Not completed 
		
		#	print component.alias
		
		for x in component.asBeg:
			x.toComponent.dependences.remove(component.alias)
			x.toComponent.asEnd.remove(x)
			self.NetworkScene.removeItem(x)
		for x in component.asEnd:
			x.fromComponent.asBeg.remove(x)
			self.NetworkScene.removeItem(x)

		if component.group!=None:	
			component.group.removeComponent(component)
		component.CheckItem.stop()
		self.NetworkScene.removeItem(component.graphicsItem)
		self.NetworkScene.update()
		component.DirectoryItem.hide()
		
		for x in self.componentList.__iter__():
			if component.graphicsItem==x.graphicsItem:
				self.componentList.remove(component)
		self.refreshCodeFromTree()
		self.Logger.logData("Deleted the Component:: "+component.alias+" SuccessFully")
		#print self.componentList.__len__()

#
# Create the Qt application, the class, and runs the program
#
if __name__ == '__main__':
	app = QtGui.QApplication(sys.argv)
	window = TheThing()
	window.show()

	if len(sys.argv) > 1:
		window.openFile(sys.argv[1])
	ret = -1

	try:
		ret = app.exec_()
	except:
		print 'Some error happened.'

	sys.exit()

