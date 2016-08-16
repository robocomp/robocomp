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

import sys, time, traceback, os, math, random, threading, time
import Ice


from PyQt4 import QtCore, QtGui, Qt
import ui_formManager,rcmanagerConfig

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

initDir=os.getcwd()

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
		
		self.simulatorTimer=QtCore.QTimer()

		self.connectionBuilder=rcmanagerConfig.connectionBuilder(self,self.Logger)##This will take care of connection building between components

		
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

		self.connect(self.graphTree.CompoPopUpMenu.ActionDelete,QtCore.SIGNAL("triggered(bool)"),self.deleteSelectedComponent)
		self.connect(self.graphTree.CompoPopUpMenu.ActionUp,QtCore.SIGNAL("triggered(bool)"),self.upSelectedComponent)
		self.connect(self.graphTree.CompoPopUpMenu.ActionAddToGroup,QtCore.SIGNAL("triggered(bool)"),self.addComponentToGroup)
		self.connect(self.graphTree.CompoPopUpMenu.ActionDown,QtCore.SIGNAL("triggered(bool)"),self.downSelectedComponent)
		self.connect(self.graphTree.CompoPopUpMenu.ActionNewConnection,QtCore.SIGNAL("triggered(bool)"),self.BuildNewConnection)
		self.connect(self.graphTree.CompoPopUpMenu.ActionControl,QtCore.SIGNAL("triggered(bool)"),self.controlComponent)
		self.connect(self.graphTree.CompoPopUpMenu.ActionRemoveFromGroup,QtCore.SIGNAL("triggered(bool)"),self.componentRemoveFromGroup)
		self.connect(self.graphTree.CompoPopUpMenu.ActionUpGroup,QtCore.SIGNAL("triggered(bool)"),self.upGroup)
		self.connect(self.graphTree.CompoPopUpMenu.ActionDownGroup,QtCore.SIGNAL("triggered(bool)"),self.downGroup)
		

		self.connect(self.UI.toolButton_2,QtCore.SIGNAL("clicked()"),self.searchEnteredAlias)
		self.connect(self.SaveWarning,QtCore.SIGNAL("save()"),self.saveXmlFile)
		self.connect(self.UI.toolButton_3,QtCore.SIGNAL("clicked()"),self.refreshTreeFromCode)
		self.connect(self.UI.toolButton_4,QtCore.SIGNAL("clicked()"),self.addNetworkTempl)
		self.connect(self.UI.toolButton_5,QtCore.SIGNAL("clicked()"),self.addComponentTempl)
		self.connect(self.UI.toolButton_6,QtCore.SIGNAL("clicked()"),self.refreshCodeFromTree)
		self.connect(self.UI.toolButton_9,QtCore.SIGNAL("clicked()"),self.editorFontSettings)
		self.connect(self.UI.toolButton_10,QtCore.SIGNAL("clicked()"),self.getNetworkSetting)
		self.connect(self.UI.toolButton,QtCore.SIGNAL("clicked()"),self.addNewComponent)
		self.Logger.logData("Tool Started")

	##The following function beginning with hover is for showing help of each Button
	def hoverAddComponent(self):
		self.UI.statusbar.showMessage("Add a new Component",3000)
	def hoverXmlSettings(self):
		self.UI.statusbar.showMessage("Edit Xml Settings",3000)
	def hoverRefreshFromTree(self):
		self.UI.statusbar.showMessage("Update the XML code from network Tree",3000)
	def hoverNetworkTreeSettings(self):
		self.UI.statusbar.showMessage("Edit Network Tree Settings",3000)
	def hoverPrintDefaultNode(self):
		self.UI.statusbar.showMessage("Add New Node",3000)
	def hoverPrintDefaultSettings(self):
		self.UI.statusbar.showMessage("Add Settings Element",3000)
	def hoverRefreshFromXml(self):
		self.UI.statusbar.showMessage("Update the Tree From the Xml Code",3000)

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
		string=rcmanagerConfig.getXmlFromNetwork(self.networkSettings,self.componentList,self.Logger)
		self.CodeEditor.setText(string)
		self.Logger.logData("Code Updated SucceFully from the graph")
	def refreshTreeFromCode(self):#This will refresh the code (Not to file)and draw the new tree
		#print "Refreshing"
		try:
			List,Settings=rcmanagerConfig.getDataFromString(str(self.CodeEditor.text()),self.Logger)
		except Exception,e:
			self.Logger.logData("Error while updating tree from Code::"+str(e), "R")
		else:
			self.removeAllComponents()
			self.NetworkScene.clear()
			self.networkSettings=Settings
			self.componentList=List
			try :
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
			self.UI.lineEdit.clear()
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
				if dist2 < self.networkSettings.spring_length: dist2 = self.networkSettings.spring_length
				force = self.networkSettings.field_force_multiplier / dist2
				force_x += force * math.cos(angle)
				force_y += force * math.sin(angle)

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
		self.refreshTreeFromCode()

	def setAllGraphicsData(self):
		for x in self.componentList:
			x.setGraphicsData()
	def StatusBarFileNameWrite(self,string):
		Label=QtGui.QLabel(string)
		self.UI.statusbar.addWidget(Label)
	def saveXmlFile(self):##To save the entire treesetting into a xml file::Unfinished
		try:
			saveFileName=QtGui.QFileDialog.getSaveFileName(self,'Save File',initDir,'*.xml')
			string=rcmanagerConfig.getXmlFromNetwork(self.networkSettings,self.componentList)
			try:
				file = open(saveFileName, 'w')
			except:
				raise Exception("Can't Open"+saveFileName)
			rcmanagerConfig.writeToFile(file,string)

			self.Logger.logData("Saved to File "+saveFileName+" ::SuccessFull")
		except:
			self.Logger.logData("Saving to File"+saveFileName+" ::Failed","R")
	
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
	
		component=rcmanagerConfig.CompInfo(view=self.graphTree,mainWindow=self)
		component.CheckItem.setLogger(self.Logger)
		self.componentList.append(component)
		component.x=pos.x()
		component.y=pos.y()
		component.graphicsItem.setPos(pos)
		self.NetworkScene.addItem(component.graphicsItem)
		self.refreshCodeFromTree()
		component.DirectoryItem.setParent(self.UI.scrollAreaWidgetContents)
		self.UI.verticalLayout.insertWidget(self.UI.verticalLayout.count()-1,component.DirectoryItem)

	def deleteComponent(self,component):##This will delete the component Not completed 
		
		#	print component.alias
		
		for x in component.asBeg:
			x.toComponent.dependences.remove(component.alias)
			x.toComponent.asEnd.remove(x)
			self.NetworkScene.removeItem(x)
		for x in component.asEnd:
			x.fromComponent.asBeg.remove(x)
			self.NetworkScene.removeItem(x)

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


if __name__ == '__main__':
	app = QtGui.QApplication(sys.argv)
	window=MainClass()
	window.show()
	if sys.argv.__len__()>1:
		try:
			if sys.argv.__len__()>2:
				raise Exception("Only one arg allowed:: Eg\n rcmanager FileName")
			window.filePath=sys.argv[1]
			window.openXmlFile(terminalArg=True)
		except Exception,e:
			print "helo"+str(e)
			sys.exit()
	try:
		ret = app.exec_()
	except:
		print 'Some error happened.'

	sys.exit()
