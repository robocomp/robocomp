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
#

#
# CODE BEGINS
#

import sys, time, traceback, os, math, random, threading, time, signal
import Ice

from PyQt4 import QtCore, QtGui, Qt
import rcmanagerConfig

CustomMainWindow = uic.loadUiType("formManager.ui")[0]  # Load the UI

try:
	_fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
	def _fromUtf8(s):
		return s

initDir = os.getcwd()

sys.path.append('.')
sys.path.append('/opt/robocomp/bin')


class MainClass(QtGui.QMainWindow, CustomMainWindow):
	"""docstring for MainClass"""
	def __init__(self, arg=None):
		super(MainClass, self).__init__(arg)
		self.currentComponent=None
		self.setWindowIcon(QtGui.QIcon(QtGui.QPixmap("/opt/robocomp/share/rcmanager/drawing_green.png")))
		self.showMaximized()
		self.componentList=[]
		self.networkSettings=rcmanagerConfig.NetworkValues()
		


		self.setupUi(self)
		self.tabWidget.removeTab(0)
		self.Logger=rcmanagerConfig.Logger(self.textBrowser)

		self.SaveWarning=rcmanagerConfig.SaveWarningDialog(self)
		
		self.NetworkScene=rcmanagerConfig.ComponentScene(self)##The graphicsScene
		self.graphTree = rcmanagerConfig.ComponentTree(self.frame,mainclass=self)##The graphicsNode
		self.NetworkScene.setSceneRect(-15000, -15000, 30000, 30000)
		self.graphTree.setScene(self.NetworkScene)
		
		
		self.graphTree.setObjectName(_fromUtf8("graphicsView"))
		self.gridLayout_8.addWidget(self.graphTree,0,0,1,1)
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

		self.CodeEditor = rcmanagerConfig.CodeEditor.get_code_editor(self.tab_2)
		self.verticalLayout_2.addWidget(self.CodeEditor)

		#The small widget which appears when we right click on a node in tree
		
		self.nodeDetailDisplayer=rcmanagerConfig.ShowItemDetails(self.graphTree)

		#Setting up the connection
		self.setupActions()

		#self.textEdit=QtGui.QTextEdit()#Temp
		#self.tabWidget_2.addTab(self.textEdit,"Helllo")#Temp
		#print "Count is "+ str(self.verticalLayout.count())
		#self.toolButton_6.setMouseTracking(True)
		
	def setupActions(self):##To setUp connection like saving,opening,etc
		self.connect(self.simulatorTimer,QtCore.SIGNAL("timeout()"),self.simulate)
		#self.connect(self.toolButton,QtCore.SIGNAL("hovered()"),self.hoverAddComponent)
		#self.connect(self.toolButton_9,QtCore.SIGNAL("hovered()"),self.hoverXmlSettings)
		#self.connect(self.toolButton_5,QtCore.SIGNAL("hovered()"),self.hoverPrintDefaultNode)
		#self.connect(self.toolButton_4,QtCore.SIGNAL("hovered()"),self.hoverPrintDefaultSettings)
		#self.connect(self.toolButton_3,QtCore.SIGNAL("hovered()"),self.hoverRefreshFromXml)
		#self.connect(self.toolButton_10,QtCore.SIGNAL("hovered()"),self.hoverNetworkTreeSettings)
		#self.connect(self.toolButton_6,QtCore.SIGNAL("hovered()"),self.hoverRefreshFromTree)
		self.connect(self.actionSet_Log_File,QtCore.SIGNAL("triggered(bool)"),self.setLogFile)

		self.connect(self.tabWidget,QtCore.SIGNAL("currentChanged(int)"),self.tabIndexChanged)
		
		# File menu buttons
		self.connect(self.actionSave,QtCore.SIGNAL("triggered(bool)"),self.saveXmlFile)
		self.connect(self.actionOpen,QtCore.SIGNAL("triggered(bool)"),self.openXmlFile)
		self.connect(self.actionExit,QtCore.SIGNAL("triggered(bool)"),self.exitRcmanager)
		
		# Edit menu buttons 
		self.connect(self.actionSetting,QtCore.SIGNAL("triggered(bool)"),self.rcmanagerSetting)
		
		# View menu buttons 
		self.connect(self.actionLogger,QtCore.SIGNAL("triggered(bool)"),self.toggleLoggerView)
		self.connect(self.actionComponent_List,QtCore.SIGNAL("triggered(bool)"),self.toggleComponentListView)
		self.connect(self.actionFull_Screen,QtCore.SIGNAL("triggered(bool)"),self.toggleFullScreenView)
			
		self.connect(self.actionON,QtCore.SIGNAL("triggered(bool)"),self.simulatorOn)
		self.connect(self.actionOFF,QtCore.SIGNAL("triggered(bool)"),self.simulatorOff)
		self.connect(self.actionSetting_2,QtCore.SIGNAL("triggered(bool)"),self.simulatorSettings)
		self.connect(self.actionSetting_3,QtCore.SIGNAL("triggered(bool)"),self.controlPanelSettings)
		self.connect(self.actionSetting_4,QtCore.SIGNAL("triggered(bool)"),self.editorSettings)
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

		self.connect(self.toolButton_2,QtCore.SIGNAL("clicked()"),self.searchEnteredAlias)
		self.connect(self.toolButton_7,QtCore.SIGNAL("clicked()"),self.simulatorOn)
		self.connect(self.toolButton_8,QtCore.SIGNAL("clicked()"),self.simulatorOff)

		self.connect(self.SaveWarning,QtCore.SIGNAL("save()"),self.saveXmlFile)
		self.connect(self.toolButton_3,QtCore.SIGNAL("clicked()"),self.refreshTreeFromCode)
		self.connect(self.toolButton_4,QtCore.SIGNAL("clicked()"),self.addNetworkTempl)
		self.connect(self.toolButton_5,QtCore.SIGNAL("clicked()"),self.addComponentTempl)
		self.connect(self.toolButton_6,QtCore.SIGNAL("clicked()"),self.refreshCodeFromTree)
		self.connect(self.toolButton_9,QtCore.SIGNAL("clicked()"),self.editorFontSettings)
		#self.connect(self.toolButton_10,QtCore.SIGNAL("clicked()"),self.getNetworkSetting)(Once finished Uncomment this)
		self.connect(self.toolButton,QtCore.SIGNAL("clicked()"),self.addNewComponent)
		
		self.Logger.logData("Tool started")
		
	# View menu functions begin 
	
	def toggleLoggerView(self):
		if self.actionLogger.isChecked():
			self.dockWidget.show()
			self.actionFull_Screen.setChecked(False)
		else:
			self.dockWidget.hide()
			self.actionFull_Screen.setChecked(not self.actionComponent_List.isChecked())
		
	def toggleComponentListView(self):
		if self.actionComponent_List.isChecked():
			self.dockWidget_2.show()
			self.actionFull_Screen.setChecked(False)
		else:
			self.dockWidget_2.hide()
			self.actionFull_Screen.setChecked(not self.actionLogger.isChecked())
			
	def toggleFullScreenView(self):
		if self.actionFull_Screen.isChecked():
			self.actionLogger.setChecked(False)
			self.actionComponent_List.setChecked(False)
			
			self.toggleLoggerView()
			self.toggleComponentListView()
		else:
			self.actionLogger.setChecked(True)
			self.actionComponent_List.setChecked(True)
			
			self.toggleLoggerView()
			self.toggleComponentListView()
					
	# View menu functions end
		
	def getFreq(self):
		comp=self.graphTree.CompoPopUpMenu.currentComponent.parent 
		comp.CheckItem.getFreq()
		
	def EditSelectedComponent(self):	
		self.tabWidget.setCurrentIndex(1)
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
			self.Logger.logData("No group","R")	
	def downGroup(self):
		component=self.graphTree.CompoPopUpMenu.currentComponent.parent
		if component.group!=None:
			component.group.downGroupComponents(self.Logger)
		else:
			self.Logger.logData("No group","R")	
			
	def componentRemoveFromGroup(self):
		component=self.graphTree.CompoPopUpMenu.currentComponent.parent
		if component.group!=None:
			component.group.removeComponent(component)
			self.NetworkScene.update()
			self.refreshCodeFromTree()
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
		index=self.tabWidget.currentIndex()
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
			self.Logger.logData("New font set for code editor")
			
	def refreshCodeFromTree(self):
		self.HadChanged=True
		string=rcmanagerConfig.getXmlFromNetwork(self.networkSettings,self.componentList,self.Logger)
		self.CodeEditor.setText(string)
		self.Logger.logData("Code updated successfully from the graph")
		self.centerAlignGraph()	
		
	def refreshTreeFromCode(self,firstTime=False):#This will refresh the code (Not to file)and draw the new tree
		#print "Refreshing"
		try:
			List,Settings=rcmanagerConfig.getDataFromString(str(self.CodeEditor.text()),self.Logger)
		except Exception,e:
			self.Logger.logData("Error while updating tree from code::"+str(e), "R")
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
					self.Logger.logData("File updated successfully from the code editor")
					self.refreshCodeFromTree()
					
				except Exception,e:
					self.Logger.logData("File updation from code failed "+str(e),"R")

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
				self.Logger.logData("Tree updated successfully from file")
	
		self.centerAlignGraph()	
		
	def centerAlignGraph(self):
		self.midValueHorizontal = (self.graphTree.horizontalScrollBar().maximum()+self.graphTree.horizontalScrollBar().minimum())/2
		self.midValueVertical = (self.graphTree.verticalScrollBar().maximum()+self.graphTree.verticalScrollBar().minimum())/2
		
		self.graphTree.horizontalScrollBar().setValue(self.midValueHorizontal)
		self.graphTree.verticalScrollBar().setValue(self.midValueVertical)
		
		self.verticalSlider.setValue(0)
		self.graphZoom()
			
	def copyAndUpdate(self,original,temp):
		if original.x!=temp.x or original.y!=temp.y:
			original.x=temp.x
			original.y=temp.y
			original.graphicsItem.setPos(original.x,original.y)
			original.graphicsItem.updateforDrag()	
			self.Logger.logData("Position updated of ::"+original.alias)
		
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
			alias=self.lineEdit.text()
			if alias== '':
				raise Exception("No Name Entered")
				
			else :
				x=self.searchforComponent(alias)#Write here what ever should happen then
				self.graphTree.centerOn(x.graphicsItem)
				#x.graphicsItem.setSelected(True)
			self.Logger.logData(alias+"  Found")
		except Exception, e:
			self.Logger.logData("Search error::  "+ str(e),"R")
		else:
			self.lineEdit.clear()
		finally:
			pass
			
	def haveChanged(self):#When the network have changed
		self.HadChanged=True
		
	def ipCount(self):#To find all the computer present
		self.ipList=[]##Ip listed from the xml file
		for x in self.componentList.__iter__():
			flag=False
			for y in self.ipList.__iter__():
				if y==x.Ip:
					flag=True
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
			self.Logger.logData("IpColor alloted successfully")		
			
		except Exception,e:
			raise Exception("Error during Ipcolors allocation "+str(e))
			  	

	def setComponentVariables(self):#Temperory function to be edited later
		for x in self.componentList.__iter__():
			x.View=self.graphTree
			x.mainWindow=self				
			self.connect(x,QtCore.SIGNAL("networkChanged()"),self.haveChanged)

	def setDirectoryItems(self):#This will set and draw all the directory components+I have added the job of defining a connection in here
		for x in self.componentList.__iter__():
			x.DirectoryItem.setParent(self.scrollAreaWidgetContents)
			self.verticalLayout.insertWidget(self.verticalLayout.count()-1,x.DirectoryItem)
		#print "Count is "+ str(self.verticalLayout.count())
	
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
		self.Logger.logData("Simulator ended")
		self.simulatorTimer.stop()
		
	def simulatorOn(self):
		self.Logger.logData("Simulator started")
		self.simulatorTimer.start(300)
		
	def simulate1(self):#To switch ON simulator::Unfinished
		for iterr in self.componentList:
			force_x = force_y = 0.
			for iterr2 in self.componentList:
				if iterr.alias == iterr2.alias: continue
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
				if dist2 < self.networkSettings.spring_length: 
					dist2 = self.networkSettings.spring_length
				force = self.networkSettings.field_force_multiplier / dist2
				force_x += force * math.cos(angle)
				force_y += force * math.sin(angle)

			for iterr2 in self.componentList:
				
				if iterr2.alias in iterr.dependences or iterr.alias in iterr2.dependences:
					ix = iterr.x - iterr2.x
					iy = iterr.y - iterr2.y
					angle = math.atan2(iy, ix)
					force = math.sqrt(abs((iy*iy) + (ix*ix)))      # force means distance actually
					#if force <= self.spring_length: continue       # "
					force -= self.networkSettings.spring_length                    # force means spring strain now
					force = force * self.networkSettings.hookes_constant           # now force means force :-)
					force_x -= force*math.cos(angle)
					force_y -= force*math.sin(angle)

			iterr.vel_x = (iterr.vel_x + (force_x*self.networkSettings.time_elapsed2))*self.networkSettings.roza
			iterr.vel_y = (iterr.vel_y + (force_y*self.networkSettings.time_elapsed2))*self.networkSettings.roza

		# Update positions
		for iterr in self.componentList:
			iterr.x += iterr.vel_x
			iterr.y += iterr.vel_y
		
		for iterr in self.componentList:
			#print "updating "+iterr.alias
			iterr.graphicsItem.setPos(QtCore.QPointF(iterr.x,iterr.y))
			iterr.graphicsItem.updateforDrag()

	def simulate(self): 
		optimalGap = 500
		tolerance = 200
	
		repulsionFactor = (optimalGap-tolerance)**2.1
		attractionFactor = 0
		dampingFactor=1
	
		for i in self.componentList:
		
			attractiveForceX=0
			attractiveForceY=0
			repulsiveForceX=0
			repulsiveForceY=0
			netForceX=0
			netForceY=0 
			dx=0
			dy=0
		
			for j in self.componentList:
				if i.alias == j.alias: 
					continue
				
				distance=((i.x-j.x)**2 + (i.y-j.y)**2)**0.5 

				if distance==0:
					distance=0.1
			
				attractiveForceX+=attractionFactor*(j.x-i.x)
				attractiveForceY+=attractionFactor*(j.y-i.y)
			
				repulsiveForceX+=repulsionFactor*(i.x-j.x)/(distance**3)
				repulsiveForceY+=repulsionFactor*(i.y-j.y)/(distance**3)
			
			netForceX=attractiveForceX+repulsiveForceX
			netForceY=repulsiveForceX+repulsiveForceY
		
			dx=netForceX*dampingFactor
			dy=netForceY*dampingFactor
		
			dx = int(dx)
			dy = int(dy)			
		
			i.vel_x=i.x+dx
			i.vel_y=i.y+dy 
		
		for i in self.componentList:
			i.x=i.vel_x
			i.y=i.vel_y
	
		for i in self.componentList:
			i.graphicsItem.setPos(QtCore.QPointF(i.x,i.y))
			i.graphicsItem.updateforDrag()
					
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
					self.Logger.logData("Connection from "+comp.alias+" to "+x.alias+" set")
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

	def setAconnection(self,toComponent,fromComponent):#To set these two components
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
			self.Logger.logData("Couldn't read from file")	
		self.refreshTreeFromCode(firstTime=True)
		self.HadChanged=False
		
	def setAllGraphicsData(self):
		for x in self.componentList:
			x.setGraphicsData()
			
	def StatusBarFileNameWrite(self,string):
		Label=QtGui.QLabel(string)
		self.statusbar.addWidget(Label)
		
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

			self.Logger.logData("Saved to file "+saveFileName+" ::successful")
		except Exception,e:
			self.Logger.logData("Saving to file"+saveFileName+" ::failed "+str(e),"R")
	
	#def saveTofile(fileName):#Save to this filename
	#	rcmanagerConfig.writeConfigToFile(self.networkSettings,self.componentList,fileName)
			
	def setZoom(self): ##To connect the slider motion to zooming
		self.verticalSlider.setRange(-20,20)
		self.verticalSlider.setTickInterval(0.5)
		self.verticalSlider.setValue(0)
		self.currentZoom=0
		self.verticalSlider.valueChanged.connect(self.graphZoom)
		
	def graphZoom(self):##To be called when ever we wants to zoomingfactor
		#NoAnchor
		#AnchorViewCenter
		#AnchorUnderMouse
	
		self.graphTree.setTransformationAnchor(self.graphTree.AnchorUnderMouse)
		
		new=self.verticalSlider.value()
		diff=new-self.currentZoom
		self.currentZoom=new
		zoomingfactor=math.pow(1.2,diff)
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
		component.DirectoryItem.setParent(self.scrollAreaWidgetContents)
		self.verticalLayout.insertWidget(self.verticalLayout.count()-1,component.DirectoryItem)
		self.tabWidget.setCurrentIndex(1)
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
		self.Logger.logData("Deleted the component '"+component.alias+"' successfully")
		#print self.componentList.__len__()
		
	def keyPressEvent(self, event):	
		if event.key()==Qt.Qt.Key_F5:
			self.refreshTreeFromCode()
		elif event.key()==Qt.Qt.Key_F11:
			self.actionFull_Screen.toggle()
			self.toggleFullScreenView()

if __name__ == '__main__':
	app = QtGui.QApplication(sys.argv)
	window=MainClass()
	window.show()
		
	if sys.argv.__len__()>1:
		try:
			if sys.argv.__len__()>3:
				raise Exception("Only two args allowed:: Eg\n rcmanager Filename Logfilename")
			
			if sys.argv.__len__()>2:
				if sys.argv[2].endswith(".log"):
					window.Logger.setFile(sys.argv[2])
				else:
					raise Exception("The log file should end with .log")

			if sys.argv[1].endswith(".xml"):
				window.filePath=sys.argv[1]
				window.openXmlFile(terminalArg=True)

			elif sys.argv[1].endswith(".log"):
				window.Logger.setFile(sys.argv[1])
			else:
				raise Exception("The target should be an .xml file or log File should be .log file")
			
		except Exception,e:
			print "Errorrr ::"+str(e)
			sys.exit()
	
	try:
		QtCore.QTimer.singleShot(50, window.centerAlignGraph)
		ret = app.exec_()

	except:
		print 'Some error happened.'
			
	sys.exit()
