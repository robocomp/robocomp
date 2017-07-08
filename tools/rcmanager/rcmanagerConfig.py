#!/usr/bin/env python2.7
#
#  -----------------------
#  ----- rcmanager -----
#  -----------------------
#  An ICE component manager.
#

#    Copyright (C) 2008-2010 by RoboLab - University of Extremadura
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

import libxml2, sys,threading,Ice ,time,os 
from PyQt4 import QtCore, QtGui, Qt, Qsci 

filePath = 'rcmanager.xml'
from time import localtime, strftime##To log data

ROBOCOMP = ''

try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior
Ice.loadSlice(preStr+"DifferentialRobot.ice")
import RoboCompDifferentialRobot

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)
			
class ComponentGroup():##On working condition
	def __init__(self):
		self.groupName=""
		self.groupIconFilePath=""
		self.groupIconPixmap=None
		self.Components=[]
		
	def __repr__(self):
		string=""
		string=string+self.groupName
		string=string+self.groupIconFilePath
		return stringIsUseful
		
	def setName(self,name):
		self.groupName=name
		
	def setIconFilePath(self,string):
		self.groupIconFilePath=string
		
	def readFromIconFile(self):##This will read from the iconFilePath and set the pixMap "Dont Forget to call the Function after name of file is assigned"
		try:
			if self.groupIconFilePath=="":
				raise NameError("groupIconFilePath is Null")
			self.groupIconPixmap=QtGui.QPixmap(self.groupIconFilePath)
		except Exception,e:
			raise e
			
	def addComponent(self,component):##Add Component to the group
		if self.Components.__contains__(component):
			raise NameError("Already in the group")
			return
		if component.group!=None:
			component.group.removeComponent(component)
		
		self.Components.append(component)
		component.groupName=self.groupName
		component.setGroup(self)
		component.graphicsItem.Icon=self.groupIconPixmap
		component.DirectoryItem.setIcon(self.groupIconPixmap)

	def removeComponent(self,component):#Remove the component from the group
		print "Removing compoent"
		self.Components.remove(component)
		component.group=None
		component.groupName=""
		component.graphicsItem.Icon=QtGui.QPixmap(getDefaultIconPath())
		component.DirectoryItem.setIcon(QtGui.QPixmap(getDefaultIconPath()))

	def upGroupComponents(self,Logger):
		for x in self.Components:
			upComponent(x,Logger)

	def downGroupComponents(self,Logger):
		for x in self.Components:
			downComponent(x,Logger)

##
#This is used to set The log file
##

class LogFileSetter_Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.resize(453, 84)
        self.gridLayout = QtGui.QGridLayout(Dialog)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.lineEdit = QtGui.QLineEdit(Dialog)
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))
        self.horizontalLayout.addWidget(self.lineEdit)
        self.pushButton = QtGui.QPushButton(Dialog)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem)
        self.pushButton_2 = QtGui.QPushButton(Dialog)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_2.addWidget(self.pushButton_2)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(_translate("Dialog", "Log File", None))
        self.pushButton.setText(_translate("Dialog", "Browse", None))
        self.pushButton_2.setText(_translate("Dialog", "Ok", None))

class LogFileSetter(QtGui.QDialog):
	def  __init__(self,parent,logger):
		QtGui.QDialog.__init__(self)
		self.parent=parent
		self.logger=logger
		self.UI=LogFileSetter_Ui_Dialog()
		self.UI.setupUi(self)
		self.connect(self.UI.pushButton,QtCore.SIGNAL("clicked()"),self.browse)
		self.connect(self.UI.pushButton_2,QtCore.SIGNAL("clicked()"),self.ok)
		
	def setFile(self):
		self.show()
		self.UI.lineEdit.setText(self.logger.filename)
		
	def browse(self):
		filename=QtGui.QFileDialog.getSaveFileName(self,'Save File',os.getcwd(),'*.log')
		filename=str(filename)
		if filename.endswith(".log")==False:
			filename=filename+".log"
		self.UI.lineEdit.setText(filename)
		
	def ok(self):
		self.logger.setFile(str(self.UI.lineEdit.text()))
		self.close()
		
class Logger():##This will be used to log data
	def __init__(self,logArea,file=None):
		self.logArea=logArea
		self.file=file
		self.fileWrite=False
		self.filename=""
		
	def logData(self,text=" ",arg="G"):#To log into the textEdit widget
		if arg=="G":
			color=QtGui.QColor.fromRgb(0,255,0)
		elif arg=="R":
			color=QtGui.QColor.fromRgb(255,0,0)
		time=strftime("%Y-%m-%d %H:%M:%S", localtime())
		self.logArea.setTextColor(color)
		self.logArea.append(time +"  >>  "+ text)

		if self.fileWrite==True:
			self.file.write("\n"+time +"  >>  "+ text)
			self.file.flush()

	def setFile(self,filename):
		self.filename=filename
		self.file = open(filename, 'a')
		self.fileWrite=True
		self.logData("\nStarted Logging into "+self.filename)

##
#This Class will take care of the Building process of a new group..THe first class is its GUI. second class is its backbone
##

class GroupBuilder_Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.resize(543, 135)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Dialog.sizePolicy().hasHeightForWidth())
        Dialog.setSizePolicy(sizePolicy)
        self.gridLayout = QtGui.QGridLayout(Dialog)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label = QtGui.QLabel(Dialog)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout.addWidget(self.label)
        self.lineEdit = QtGui.QLineEdit(Dialog)
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))
        self.horizontalLayout.addWidget(self.lineEdit)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_2 = QtGui.QLabel(Dialog)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_3.addWidget(self.label_2)
        self.lineEdit_2 = QtGui.QLineEdit(Dialog)
        self.lineEdit_2.setObjectName(_fromUtf8("lineEdit_2"))
        self.horizontalLayout_3.addWidget(self.lineEdit_2)
        self.pushButton = QtGui.QPushButton(Dialog)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout_3.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem)
        self.pushButton_2 = QtGui.QPushButton(Dialog)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_4.addWidget(self.pushButton_2)
        self.pushButton_3 = QtGui.QPushButton(Dialog)
        self.pushButton_3.setAutoDefault(False)
        self.pushButton_3.setDefault(True)
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))
        self.horizontalLayout_4.addWidget(self.pushButton_3)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(_translate("Dialog", "Group Builder", None))
        self.label.setText(_translate("Dialog", "Group Name", None))
        self.label_2.setText(_translate("Dialog", "Icon File         ", None))
        self.pushButton.setText(_translate("Dialog", "Browse", None))
        self.pushButton_2.setText(_translate("Dialog", "Cancel", None))
        self.pushButton_3.setText(_translate("Dialog", "Ok", None))

class GroupBuilder(QtGui.QDialog):
	def  __init__(self,parent,logger):
		QtGui.QDialog.__init__(self)
		self.parent=parent
		self.logger=logger
		self.UI=GroupBuilder_Ui_Dialog()
		self.UI.setupUi(self)
		self.connect(self.UI.pushButton_3,QtCore.SIGNAL("clicked()"),self.SaveGroup)
		self.connect(self.UI.pushButton_2,QtCore.SIGNAL("clicked()"),self.cancel)
		self.connect(self.UI.pushButton,QtCore.SIGNAL("clicked()"),self.browseIcon)
		self.group=None
		self.build=False
		
	def startBuildGroup(self,networkSettings):
		self.build=False
		self.networkSettings=networkSettings
		self.group=ComponentGroup()
		self.show()
		
	def SaveGroup(self):
		self.group.setName(self.UI.lineEdit.text())
		self.group.setIconFilePath(self.UI.lineEdit_2.text())
		self.group.readFromIconFile()
		self.networkSettings.Groups.append(self.group)
		self.parent.refreshCodeFromTree()
		self.logger.logData("New Group::"+self.UI.lineEdit.text()+" Build")
		self.UI.lineEdit.setText("")
		self.UI.lineEdit_2.setText("")
		self.build=True
		self.close()
		
	def cancel(self):
		self.build=False
		self.close()
		
	def browseIcon(self):
		self.UI.lineEdit_2.setText(QtGui.QFileDialog.getOpenFileName(self,'Set Group Icon',os.getcwd(),"Image Files (*.png *.jpg *.bmp)"))

	def closeEvent(self,event):
		QtGui.QDialog.closeEvent(self,event)
		if self.build==False:
			self.logger.logData("Group Building Canceled by User","R")
			self.UI.lineEdit.setText("")
			self.UI.lineEdit_2.setText("")
		self.build=False

##
#This Will take care of adding a component into a particular group..The first class is its GUI and the second one is its main thing
##

class AddToGroup_Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.resize(282, 394)
        self.gridLayout = QtGui.QGridLayout(Dialog)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.listWidget = QtGui.QListWidget(Dialog)
        self.listWidget.setObjectName(_fromUtf8("listWidget"))
        self.horizontalLayout.addWidget(self.listWidget)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.pushButton_2 = QtGui.QPushButton(Dialog)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_2.addWidget(self.pushButton_2)
        self.pushButton = QtGui.QPushButton(Dialog)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout_2.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
    	Dialog.setWindowTitle(_translate("Dialog", "Group Selector", None))
        self.pushButton_2.setText(_translate("Dialog", "Cancel", None))
        self.pushButton.setText(_translate("Dialog", "Ok", None))

class GroupSelector(QtGui.QDialog):
	def __init__(self,parent,logger):
		self.logger=logger
		self.parent=parent
		QtGui.QDialog.__init__(self)
		self.UI=AddToGroup_Ui_Dialog()
		self.UI.setupUi(self)
		self.connect(self.UI.pushButton_2,QtCore.SIGNAL("clicked()"),self.cancel)
		self.connect(self.UI.pushButton,QtCore.SIGNAL("clicked()"),self.selected)
		self.groupList=None
		self.component=None
		self.groupAdded=False
		
	def openSelector(self,component,groupList):
		self.groupAdded=False
		self.groupList=groupList
		self.component=component
		self.UI.listWidget.clear()
		for x in self.groupList:
			item=QtGui.QListWidgetItem(QtGui.QIcon(x.groupIconPixmap),x.groupName)
			self.UI.listWidget.addItem(item)
		self.show()
		self.compoent=component
		self.groupList=groupList
		
	def cancel(self):
		self.groupAdded=False
		self.close()	

	def selected(self):
		string=self.UI.listWidget.currentItem().text()
		for x in self.groupList:
			if x.groupName==string:
				x.addComponent(self.component)
		self.UI.listWidget.clear()
		self.groupAdded=True
		self.logger.logData("Component ::"+self.component.alias+" Added to group "+self.component.groupName )
		self.parent.refreshCodeFromTree()
		self.close()
		
	def closeEvent(self,event):
		if self.groupAdded==False:
			QtGui.QDialog.closeEvent(self,event)
			self.logger.logData("Adding to group Cancelled by User","R")
			self.UI.listWidget.clear()
				
##
#This is inherited tool Button ..Main reason was to show its purpose while hovering the button(Not using anymore)
##

class toolButton(QtGui.QToolButton):
	def  __init__(self,parent):
		QtGui.QToolButton.__init__(self,parent)
		self.setMouseTracking(True)
	def enterEvent(self,event):
		self.emit(QtCore.SIGNAL("hovered()"))
		QtGui.QToolButton.enterEvent(self,event)

##
#This will take care of the Building the connection between nodes..The first class is its GUI.the second one is the main thing.
##

class ConnectionBuilder_Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.resize(434, 138)
        self.gridLayout = QtGui.QGridLayout(Dialog)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_2 = QtGui.QLabel(Dialog)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_2.addWidget(self.label_2)
        self.lineEdit_2 = QtGui.QLineEdit(Dialog)
        self.lineEdit_2.setObjectName(_fromUtf8("lineEdit_2"))
        self.horizontalLayout_2.addWidget(self.lineEdit_2)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label = QtGui.QLabel(Dialog)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout.addWidget(self.label)
        self.lineEdit = QtGui.QLineEdit(Dialog)
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))
        self.horizontalLayout.addWidget(self.lineEdit)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem)
        self.pushButton_2 = QtGui.QPushButton(Dialog)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_3.addWidget(self.pushButton_2)
        self.pushButton = QtGui.QPushButton(Dialog)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout_3.addWidget(self.pushButton)
        self.gridLayout.addLayout(self.horizontalLayout_3, 1, 0, 1, 1)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(_translate("Dialog", "Connection Builder", None))
        self.label_2.setText(_translate("Dialog", "From", None))
        self.label.setText(_translate("Dialog", "To", None))
        self.pushButton_2.setText(_translate("Dialog", "Cancel", None))
        self.pushButton.setText(_translate("Dialog", "Ok", None))

class connectionBuilder(QtGui.QDialog):## This is used to set connection between two different dialogs
	def __init__(self,parent,logger):
		QtGui.QDialog.__init__(self)
		
		self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)		
		self.logger=logger
		self.parent=parent
		self.UI=ConnectionBuilder_Ui_Dialog()
		self.UI.setupUi(self)
		self.connection=None
		self.fromComponent=None
		self.toComponent=None
		self.BuildingStatus=False
		self.connect(self.UI.pushButton,QtCore.SIGNAL("clicked()"),self.SaveConnection)
		self.connect(self.UI.pushButton_2,QtCore.SIGNAL("clicked()"),self.closeWithoutSaving)
		
	def buildNewConnection(self):
		self.connection=NodeConnection()
		self.BuildingStatus=True
		
	def setBeg(self,component):
		self.UI.lineEdit_2.setText(component.alias)
		self.connection.fromComponent=component
		self.fromComponent=component
		
	def setEnd(self,component):
		self.UI.lineEdit.setText(component.alias)
		self.connection.toComponent=component
		self.toComponent=component
		
	def SaveConnection(self):
		self.toComponent.dependences.append(self.fromComponent.alias)
		self.toComponent.asEnd.append(self.connection)
		self.fromComponent.asBeg.append(self.connection)
		self.parent.NetworkScene.addItem(self.connection)
		self.close()
		self.parent.NetworkScene.update()
		self.parent.refreshCodeFromTree()
		self.logger.logData("Connection Made From "+self.fromComponent.alias+" to "+self.toComponent.alias)
		self.BuildingStatus=False
		self.UI.lineEdit.setText("")
		self.UI.lineEdit_2.setText("")
		
	def closeWithoutSaving(self):
		self.close()
		self.UI.lineEdit.setText("")
		self.UI.lineEdit_2.setText("")

##
#This will takes care of selecting the rcmanger tool settings..The first class is about UI.second is main thing..
##

class NetworkSettings_Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.resize(561, 263)
        self.gridLayout = QtGui.QGridLayout(Dialog)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem)
        self.pushButton_2 = QtGui.QPushButton(Dialog)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_4.addWidget(self.pushButton_2)
        self.pushButton_3 = QtGui.QPushButton(Dialog)
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))
        self.horizontalLayout_4.addWidget(self.pushButton_3)
        self.pushButton = QtGui.QPushButton(Dialog)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout_4.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.gridLayout.addLayout(self.verticalLayout, 1, 0, 1, 1)
        self.formLayout = QtGui.QFormLayout()
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setObjectName(_fromUtf8("formLayout"))
        self.label = QtGui.QLabel(Dialog)
        self.label.setObjectName(_fromUtf8("label"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.LabelRole, self.label)
        self.doubleSpinBox = QtGui.QDoubleSpinBox(Dialog)
        self.doubleSpinBox.setObjectName(_fromUtf8("doubleSpinBox"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.doubleSpinBox)
        self.label_2 = QtGui.QLabel(Dialog)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label_2)
        self.doubleSpinBox_2 = QtGui.QDoubleSpinBox(Dialog)
        self.doubleSpinBox_2.setObjectName(_fromUtf8("doubleSpinBox_2"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.doubleSpinBox_2)
        self.label_3 = QtGui.QLabel(Dialog)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label_3)
        self.doubleSpinBox_3 = QtGui.QDoubleSpinBox(Dialog)
        self.doubleSpinBox_3.setObjectName(_fromUtf8("doubleSpinBox_3"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.doubleSpinBox_3)
        self.label_4 = QtGui.QLabel(Dialog)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.formLayout.setWidget(3, QtGui.QFormLayout.LabelRole, self.label_4)
        self.doubleSpinBox_4 = QtGui.QDoubleSpinBox(Dialog)
        self.doubleSpinBox_4.setObjectName(_fromUtf8("doubleSpinBox_4"))
        self.formLayout.setWidget(3, QtGui.QFormLayout.FieldRole, self.doubleSpinBox_4)
        spacerItem1 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.formLayout.setItem(4, QtGui.QFormLayout.FieldRole, spacerItem1)
        self.gridLayout.addLayout(self.formLayout, 0, 0, 1, 1)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(_translate("Dialog", "NetworkSettings", None))
        self.pushButton_2.setText(_translate("Dialog", "Apply", None))
        self.pushButton_3.setText(_translate("Dialog", "Cancel", None))
        self.pushButton.setText(_translate("Dialog", "OK", None))
        self.label.setText(_translate("Dialog", "Connection Modularity", None))
        self.label_2.setText(_translate("Dialog", "Node Modularity", None))
        self.label_3.setText(_translate("Dialog", "X-Multiplication factor", None))
        self.label_4.setText(_translate("Dialog", "Y-Multiplication factor", None))
		
class NetworkSettings(QtGui.QDialog):#This will show a dialog window for selecting the rcmanager tool settings
	def __init__(self,parent=None):
		QtGui.QDialog.__init__(self)
		self.parent=parent
		self.UI=NetworkSettings_Ui_Dialog()			
		self.UI.setupUi(self)
		self.setting=None
		
	def setData(self,setting):
		self.setting=setting

##
#This will take care of the Save warning and stuff to avoid accidental quiting without saving ..First class is for UI..Second one takes care of the main stuffs..
##

class SaveWarning_Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.resize(493, 87)
        self.horizontalLayout = QtGui.QHBoxLayout(Dialog)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.pushButton = QtGui.QPushButton(Dialog)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtGui.QPushButton(Dialog)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8("share/rcmanager/1465394415_floppy.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_2.setIcon(icon)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout.addWidget(self.pushButton_2)
        self.pushButton_3 = QtGui.QPushButton(Dialog)
        self.pushButton_3.setAutoDefault(True)
        self.pushButton_3.setDefault(True)
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))
        self.horizontalLayout.addWidget(self.pushButton_3)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(_translate("Dialog", "Save Warning", None))
        self.pushButton.setText(_translate("Dialog", "Don\'t Save", None))
        self.pushButton_2.setText(_translate("Dialog", "Save", None))
        self.pushButton_3.setText(_translate("Dialog", "Cancel", None))

class SaveWarningDialog(QtGui.QDialog):#To be used as a warning window while deleting existing tree without saving
	def __init__(self,parent=None):
		QtGui.QDialog.__init__(self)
		self.parent=parent
		self.UI=SaveWarning_Ui_Dialog()
		self.UI.setupUi(self)
		self.connect(self.UI.pushButton_2,QtCore.SIGNAL("clicked()"),self.save)
		self.connect(self.UI.pushButton,QtCore.SIGNAL("clicked()"),self.dontSave)
		self.connect(self.UI.pushButton_3,QtCore.SIGNAL("clicked()"),self.cancel)
		self.setModal(True)
		self.Status="C"
		
	def decide(self):
		self.exec_()
		return self.Status
		
	def save(self):
		self.close()
		self.Status="S"
		
	def dontSave(self):
		self.close()
		self.Status="D"
		
	def cancel(self):
		self.close()
		self.Status="C"

##
#This will takes care about the code editing part of the software
##

class CodeEditor(Qsci.QsciScintilla):#For the dynamic code editing (Widget )
	def __init__(self,parent=None):
		Qsci.QsciScintilla.__init__(self,parent)

		#Setting default font
		self.font=QtGui.QFont()
		self.font.setFamily('Courier')
		self.font.setFixedPitch(True)
		self.font.setPointSize(10)
		self.setFont(self.font)
		self.setMarginsFont(self.font)

		# Margin 0 is used for line numbers
		fontmetrics =QtGui.QFontMetrics(self.font)
		self.setMarginsFont(self.font)
		self.setMarginWidth(0,fontmetrics.width("0000") + 6)
		self.setMarginLineNumbers(0, True)
		self.setMarginsBackgroundColor(QtGui.QColor("#cccccc"))		

		#BraceMatching
		self.setBraceMatching(Qsci.QsciScintilla.SloppyBraceMatch)

		# Current line visible with special background color
		self.setCaretLineVisible(True)
		self.setCaretLineBackgroundColor(QtGui.QColor("#ffe4e4"))

		#Setting xml lexer
		lexer = Qsci.QsciLexerXML()
		lexer.setDefaultFont(self.font)
		self.setLexer(lexer)
		self.SendScintilla(Qsci.QsciScintilla.SCI_STYLESETFONT, 1, 'Courier')
	
	def on_margin_clicked(self, nmargin, nline, modifiers):
		# Toggle marker for the line the margin was clicked on
		if self.markersAtLine(nline) != 0:
			self.markerDelete(nline, self.ARROW_MARKER_NUM)
		else:
			self.markerAdd(nline, self.ARROW_MARKER_NUM)

##
#This is the graphics Item which represent a component
##

class VisualNode(QtGui.QGraphicsItem):##Visual Node GraphicsItem
	"""docstring for ClassName"""
	def __init__(self, view=None,Alias="Component",parent=None):
		QtGui.QGraphicsItem.__init__(self)
		self.parent=parent
		self.view=view
		self.Alias=Alias
		self.pos=None
		self.IpColor=QtGui.QColor.fromRgb(255,255,255)
		self.aliveStatus=False
		self.detailsShower=None
		self.setFlag(QtGui.QGraphicsItem.ItemIsMovable)
		self.setAcceptHoverEvents(True)
		self.Icon=QtGui.QPixmap(getDefaultIconPath())
		#self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable)
		self.setZValue(1)#To make sure the Nodes are always on top of connections
		
	def mouseMoveEvent(self,event):
		#self.parent.mainWindow.nodeDetailDisplayer.hide()
		QtGui.QGraphicsItem.mouseMoveEvent(self,event)
		self.updateforDrag()
		self.parent.emit(QtCore.SIGNAL("networkChanged()"))
		#print "Changed"

	def updateforDrag(self):
		for x in self.parent.asBeg.__iter__():
			end=x.toComponent
			a,b=findWhichPorts(self.parent,end)
			x.fromX,x.fromY=findPortPosition(self.parent,a)
			x.toX,x.toY=findPortPosition(end,b)
			x.prepareGeometryChange()
		for x in self.parent.asEnd.__iter__():
			beg=x.fromComponent
			a,b=findWhichPorts(beg,self.parent)
			x.fromX,x.fromY=findPortPosition(beg,a)
			x.toX,x.toY=findPortPosition(self.parent,b)
			x.prepareGeometryChange()
		self.scene().update()
		
	def mouseReleaseEvent(self,event):
		QtGui.QGraphicsItem.mouseReleaseEvent(self,event)
		self.updateforDrag(	)
		self.parent.x=self.x()
		self.parent.y=self.y()
		
	"""
	def hoverEnterEvent(self,event):
		pos=self.parent.mainWindow.graphTree.mapFromScene(self.mapToScene(QtCore.QPointF()))
		self.parent.mainWindow.nodeDetailDisplayer.showdetails(pos.x(),pos.y(),self.parent)
	def hoverLeaveEvent(self,event):
		self.parent.mainWindow.nodeDetailDisplayer.isShowing=False
		self.parent.mainWindow.nodeDetailDisplayer.hide()
	""" 
	
	def setIpColor(self,color=QtGui.QColor.fromRgb(0,255,0)):#To set the Ipcolor
		self.IpColor=color
		
	def setview(view):
		self.view=view
		
	def boundingRect(self):
		self.penWidth =2
		return QtCore.QRectF(-69,-69,138,138)
		
	def paint(self,painter,option=None,widget=None):
		self.paintMainShape(painter)
		self.drawStatus(painter)
		self.drawIcon(painter)
		self.writeAlias(painter)
		
	def setIcon(self,icon=None):
		self.Icon=icon
		
	def paintMainShape(self,painter): ##This will draw the basic shape of a node.The big square and its containing elements
		pen=QtGui.QPen(QtGui.QColor.fromRgb(self.parent.nodeColor[0],self.parent.nodeColor[1],self.parent.nodeColor[2]))
		pen.setWidth(3)
		painter.setPen(pen)
		brush=QtGui.QBrush(QtGui.QColor.fromRgb(94,94,94))
		painter.setBrush(brush)
		painter.drawRoundedRect(-49,-49,98,98,5,5)
		
		self.TextRect=QtCore.QRect(-45,-45,90,20)##The rectangle shape on which the alias name will be dispalyed
		self.statusRect=QtCore.QRect(22,10,20,20)##The rectange shape on which the status of the node will be displayed
		self.IconRect=QtCore.QRect(-45,-20,60,64)## The rectange shape on which the Icon will be displayed
		
		brush.setColor(QtGui.QColor.fromRgb(94,94,94))
		self.drawUpPort(painter,brush)##Temp
		self.drawDownPort(painter,brush)##Temp
		self.drawRightPort(painter,brush)##Temp
		self.drawLeftPort(painter,brush)##Temp
		
		brush.setColor(QtGui.QColor.fromRgb(255,255,255))
		painter.setBrush(brush)
		painter.drawRect(self.TextRect) ##Drawing the Alias display rectangle
		painter.setBrush(self.IpColor)  ## Drawing the Icon display rectangle
		painter.drawRect(self.IconRect)
		
		painter.setPen(QtGui.QColor.fromRgb(0,0,0))
		
	def drawUpPort(self,painter,brush):##The bulging arcs will can be calleds as the port because all connection starts from there
		painter.setBrush(brush)
		painter.drawChord(-10,-59,20,20,0,2880)
	
	def drawDownPort(self,painter,brush):
		painter.setBrush(brush)
		painter.drawChord(-10,39,20,20,0,-2880)

	def drawRightPort(self,painter,brush):
		painter.setBrush(brush)
		painter.drawChord(39,-10,20,20,1440,-2880)

	def drawLeftPort(self,painter,brush):
		painter.setBrush(brush)
		painter.drawChord(-59,-10,20,20,1440,2880)

	def drawIcon(self,painter):#Draw the Icon representing the purpose and draw its 
		painter.setBrush(self.IpColor)  ## Drawing the Icon display rectangle
		painter.drawRect(self.IconRect)
		painter.drawPixmap(-45,-20,60,60,self.Icon,0,0,0,0)
		
	def writeAlias(self,painter):##Draw the alias
		painter.drawText(self.TextRect,1," "+self.Alias) 	##Drawing the Alias name
		
	def drawStatus(self,painter):
		if self.parent.status:
			brush=QtGui.QBrush(QtGui.QColor.fromRgb(0,255,0)) ##Drawing the Status display rectangle
			painter.setBrush(brush)
			painter.drawRect(self.statusRect)
		else:
			brush=QtGui.QBrush(QtGui.QColor.fromRgb(255,0,0)) ##Drawing the Status display rectangle
			painter.setBrush(brush)
			painter.drawRect(self.statusRect)

def findPortPosition(Component,Port):#PORT can be "U"or "D" or "R" or "L"
		ItemPoint=QtCore.QPointF()
		if Port=="U":
			ItemPoint.setX(0)
			ItemPoint.setY(-54)
			X=QtGui.QGraphicsItem.mapToScene(Component.graphicsItem,ItemPoint).x()
			Y=QtGui.QGraphicsItem.mapToScene(Component.graphicsItem,ItemPoint).y()
			return X,Y
		elif Port=="D":
			ItemPoint.setX(0)
			ItemPoint.setY(54)
			X=QtGui.QGraphicsItem.mapToScene(Component.graphicsItem,ItemPoint).x()
			Y=QtGui.QGraphicsItem.mapToScene(Component.graphicsItem,ItemPoint).y()
			return X,Y
		elif Port=="R":
			ItemPoint.setX(54)
			ItemPoint.setY(0)
			X=QtGui.QGraphicsItem.mapToScene(Component.graphicsItem,ItemPoint).x()
			Y=QtGui.QGraphicsItem.mapToScene(Component.graphicsItem,ItemPoint).y()
			return X,Y	
		elif Port=="L":
			ItemPoint.setX(-54)
			ItemPoint.setY(0)
			X=QtGui.QGraphicsItem.mapToScene(Component.graphicsItem,ItemPoint).x()
			Y=QtGui.QGraphicsItem.mapToScene(Component.graphicsItem,ItemPoint).y()
			return X,Y

def findWhichPorts(fromComponent,toComponent):#This will select out of the 4 ports which have to be connected to connection		
		
		a=fromComponent.graphicsItem.mapToScene(QtCore.QPointF())
		b=toComponent.graphicsItem.mapToScene(QtCore.QPointF())

		Line=QtCore.QLineF(b,a)
		
		angle=Line.angle()
		#print angle
		if angle<=45 or angle >315:
			return "L","R"
		elif angle>45 and angle<=135:
			return "D","U"
		elif angle>135 and angle <=225:
			return "R","L"
		elif angle>225 and angle <=315:
			return "U","D"

##
#This is another graphics Item which represents the connection between The componentts
##

class NodeConnection(QtGui.QGraphicsItem):
	def __init__(self):
		QtGui.QGraphicsItem.__init__(self)
		self.fromComponent=None##CompInfo Class
		self.toComponent=None##CompInfo Class
		self.fromX=0
		self.fromY=0
		self.toX=0
		self.toY=0
		self.color=QtGui.QColor.fromRgb(94,94,94)
		self.pen=QtGui.QPen(self.color)
		self.penWidth=2
		self.pen.setWidth(self.penWidth)
		self.setBoundingRegionGranularity(0.9)
		self.Line=None
		self.fromPoint=QtCore.QPointF()
		self.toPoint=QtCore.QPointF()	
		self.arrowLine=QtCore.QLineF()
		
	def boundingRect(self):##TO Be modified..Because error prone when vertical and horizontal
		if abs(self.toX-self.fromX)<5:
			width=5
		else:
			width=self.toX-self.fromX
		if abs(self.toY-self.fromY)<5:
			height=5
		else:
			height=self.toY-self.fromY
		self.rect=QtCore.QRectF(self.fromX,self.fromY,width,height)
		return self.rect
		
	def paint(self,painter,option=None,widget=None):
		painter.setPen(self.pen)
		self.fromPoint.setX(self.fromX)
		self.fromPoint.setY(self.fromY)
		self.toPoint.setX(self.toX)
		self.toPoint.setY(self.toY)
		self.Line=QtCore.QLineF(self.toPoint,self.fromPoint)
		painter.drawLine(self.Line)
		self.drawArrows(painter)
		#painter.drawRect(self.rect)
		
	def drawArrows(self,painter):#To 	draw the arrows in the connections::Unfinished
		V1=self.Line.unitVector()
		V1.setAngle(V1.angle()+20)
		V1.setLength(40)
		p1=V1.p2()
		V1.setAngle(V1.angle()-40)
		p2=V1.p2()
		brush=QtGui.QBrush(QtGui.QColor.fromRgb(0,0,0))
		painter.setBrush(brush)
		painter.drawPolygon(self.toPoint,p1,p2)	
		
	def hoverEnterEvent(self,Event):#Unfinished
		print "Mouse Hovering in connection from :" +self.fromComponent+" to :" +self.toPoint

##
#This class will monitor the components whether they are up or down..
##
				
class ComponentChecker(threading.Thread):#This will check the status of components
	def __init__(self,component):	
		threading.Thread.__init__(self)
		self.component=component
		self.transmitter=QtCore.QObject()
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.daemon = True
		self.reset()
		self.exit = False
		self.alive = False
		self.aPrx = None
		self.started=False
		
	def getFreq(self):
		self.mutex.lock()
		self.object=RoboCompCommonBehavior.CommonBehaviorPrx.checkedCast(self.aPrx)
		print self.object.getFreq()
		self.mutex.unlock()
		
	def setLogger(self,logger):
		self.logger=logger
		
	def haveStarted(self):
		return self.started
		
	def initializeComponent(self):#Called to set the component and to initialize the Ice proxy
		self.mutex.lock()
		try:
			ic=Ice.initialize(sys.argv)
			self.aPrx = ic.stringToProxy(self.component.endpoint)
			if self.aPrx!= None:
				self.aPrx.ice_timeout(1)
			self.started=True
		except :
			print "Error creating proxy to " + self.component.endpoint
			if len(self.component.endpoint) == 0:
				print 'please, provide an endpoint'

		self.mutex.unlock()

	def run(self):
		if self.aPrx==None:

			self.initializeComponent()

		#global global_ic
		while self.exit == False:
			if self.started:
				try:
					self.aPrx.ice_ping()
					self.mutex.lock()
					if self.alive==False:
						self.changed()	
					self.alive = True
					self.mutex.unlock()
				except:
					self.mutex.lock()
					if self.alive==True:
						self.changed()
					self.alive = False
					self.mutex.unlock()
			#self.mutex.lock()
			#print self.component.alias +" ::Status:: "+ str(self.alive)
			#self.mutex.unlock()
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
		self.mutex.lock()
		self.exit = True
		self.mutex.unlock()
		
	def runrun(self):
		if not self.isAlive(): self.start()#Note this is different isalive
		
	def changed(self):
		self.component.status=not self.alive
		self.component.graphicsItem.update()

		if self.alive==False:
			print "Component "+self.component.alias+ " UP"  ##Couldn't log into the main page..Because the logger QOWidge cannot be used in another thread..
		if self.alive==True:
			print "Component "+self.component.alias+ " Down"
		
##
#This widget is used to display the details of a component when hovering over the nodes
##

class ShowItemDetails(QtGui.QWidget):##This contains the GUI and internal process regarding the controlling of the a particular component.
	def __init__(self,parent=None):
		QtGui.QWidget.__init__(self)
		self.component=None
		self.setParent(parent)
		self.detailString=" "
		self.label=QtGui.QLabel(self)
		self.label.setGeometry(0,0,200,150)
		self.isShowing=False
		self.setAutoFillBackground(True)
		self.hide()
		
	def showdetails(self,x,y,item=None):
		self.item=item
		string=""
		string=string+"Name ::"+item.alias+"\n"
		string=string+"Group Name:: "+item.groupName+"\n"

		self.label.setText(string)
		self.setGeometry(x,y,150,150)
	  	self.isShowing=True
	  	self.show()

	#def contextMenuEvent(self,event):
	#	print "hello"
	#	GloPos=event.globalPos()
	#	self.hide()
	#	self.parent.graphTree.CompoPopUpMenu.setComponent(self.item.graphicsItem)
	#	self.CompoPopUpMenu.popup(GloPos)
		
#	
# Component information container class.This is the class which represents a component..the graphics object,directory object,monitoring thread everything is contained in the class
#

class CompInfo(QtCore.QObject):##This contain the general Information about the Components which is read from the files and created
	def __init__(self,view=None,mainWindow=None,name="Component"):
		QtCore.QObject.__init__(self)
		self.vel_x=0
		self.vel_y=0
		self.group=None
		self.groupName=""
		self.View=view
		self.mainWindow=mainWindow
		self.asEnd=[]#This is the list of connection where the node act as the ending point
		self.asBeg=[]#This is the list of connection where the node act as the beginning point
		self.endpoint = ''
		self.workingdir = ''
		self.compup = ''
		self.compdown = ''
		self.alias =name
		self.dependences = []
		self.configFile = ''
		self.x = 0##This is not reliable >>Have to fix the bug
		self.y = 0##This is not reliable >>Have to fix the bug
		self.Ip=""
		self.IconFilePath=getDefaultIconPath()
		self.status=False
		self.nodeColor=[0,0,0]
		self.tempx=0 #This will be used sometime for temporary changes
		self.tempy=0 #This will be used sometime for temporary changes

		self.CommonProxy=commonBehaviorComponent(self)
		
		self.CheckItem=ComponentChecker(self)##Checking the status of component
		self.graphicsItem=VisualNode(parent=self)
		self.DirectoryItem=DirectoryItem(parent=self,name=self.alias)
		#self.Controller=ComponentController(parent=self)
		
	def setGroup(self,group):
		self.group=group

	def __repr__(self):
		string = ''
		string = string + '[' + self.alias + ']:\n'
		string = string + 'endpoint:  \t' + self.endpoint + '\n'
		string = string + 'workingDir:\t' + self.workingdir + '\n'
		string = string + 'up:        \t' + self.compup + '\n'
		string = string + 'down:      \t' + self.compdown + '\n'
		string = string + 'x:         \t' + str(self.x) + '\n'
		string = string + 'y:         \t' + str(self.y) + '\n'
		string = string + 'Icon path: \t' + self.IconFilePath+"\n"
		return string	
		
	def setGraphicsData(self):#To set the graphicsItem datas
		self.graphicsItem.setIpColor()
		self.graphicsItem.setPos(self.x,self.y)
		self.graphicsItem.Alias=self.alias

##
#This is the inherited GraphicsView Class on which the graphicsScene is displayed
##

class  ComponentTree(QtGui.QGraphicsView):	##The widget on which we are going to draw the graphtree 
	def __init__(self,parent,mainclass):
		self.viewportAnchor=QtGui.QGraphicsView.AnchorUnderMouse
		QtGui.QGraphicsView.__init__(self,parent)
		self.connectionBuidingStatus=False
		self.mainclass=mainclass#This object is the mainClass from rcmanager Module
		self.CompoPopUpMenu=ComponentMenu(self.mainclass)
		self.BackPopUpMenu=BackgroundMenu(self.mainclass)
		
		self.lastPosition=None
		self.leftMouseButtonClicked=False
		#self.ctrlButtonClicked=False
	
	def wheelEvent(self,wheel):
		QtGui.QGraphicsView.wheelEvent(self,wheel)
		temp=self.mainclass.currentZoom
		temp+=(wheel.delta()/120)
		self.mainclass.UI.verticalSlider.setValue(temp)
		self.mainclass.graphZoom() 
	
	def contextMenuEvent(self,event):##It will select what kind of context menu should be displayed 
		#if self.ctrlButtonClicked:
		#	return
			
		if self.leftMouseButtonClicked:
			return
		
		GloPos=event.globalPos()
		pos=event.pos()
		item=self.itemAt(pos)

		if isinstance(item,VisualNode):
			self.CompoPopUpMenu.setComponent(item)
			self.CompoPopUpMenu.popup(GloPos)
		else:
			self.BackPopUpMenu.setPos(pos)
			self.BackPopUpMenu.popup(GloPos)

	"""
	def keyPressEvent(self, event):
		if event.key()==Qt.Qt.Key_Control:
			self.ctrlButtonClicked=True
	""" 

	def mousePressEvent(self,event):	
		if event.button()==Qt.Qt.LeftButton:
			self.leftMouseButtonClicked=True
			
			print "Left mouse button pressed"
			
		if event.button()==Qt.Qt.RightButton and self.leftMouseButtonClicked:
			self.setDragMode(QtGui.QGraphicsView.ScrollHandDrag)
			self.lastPosition=event.pos()
			
			print "Right mouse button pressed, entering pan mode"
			
			return 
			
		if event.button()==Qt.Qt.MidButton:
			self.setDragMode(QtGui.QGraphicsView.ScrollHandDrag)
			self.lastPosition=event.pos()
			
			print "Middle mouse button pressed, entering pan mode"
			
			return
		
		"""	
		if event.button()==Qt.Qt.RightButton and self.ctrlButtonClicked:
			self.setDragMode(QtGui.QGraphicsView.ScrollHandDrag)
			self.lastPosition=event.pos()
			return
		""" 
		
		if self.connectionBuidingStatus==True:
			pos=event.pos()
			item=self.itemAt(pos)		
			if isinstance(item,VisualNode):
				self.mainclass.connectionBuilder.setEnd(item.parent)
			self.connectionBuidingStatus=False
			
		if self.connectionBuidingStatus==False:
			QtGui.QGraphicsView.mousePressEvent(self,event)
		
	def mouseMoveEvent(self, event):
		if self.dragMode()==QtGui.QGraphicsView.ScrollHandDrag:
			self.currentPosition=event.pos()
			
			print "panning, init pos:", self.lastPosition.x(), self.lastPosition.y(), "final pos:", self.currentPosition.x(), self.currentPosition.y()
			
			dx=self.currentPosition.x()-self.lastPosition.x()
			dy=self.currentPosition.y()-self.lastPosition.y()
			self.verticalScrollBar().setValue(self.verticalScrollBar().value()-dy)
			self.horizontalScrollBar().setValue(self.horizontalScrollBar().value()-dx)
			self.lastPosition=self.currentPosition
		
		QtGui.QGraphicsView.mouseMoveEvent(self,event)
			
	"""
	def keyReleaseEvent(self, event):
		if event.key()==Qt.Qt.Key_Control:
			self.ctrlButtonClicked=False
			self.setDragMode(QtGui.QGraphicsView.NoDrag)
	"""  
			
	def mouseReleaseEvent(self, event):
		if event.button()==Qt.Qt.LeftButton:
			self.leftMouseButtonClicked=False
			self.setDragMode(QtGui.QGraphicsView.NoDrag)
			
			print "Left mouse button released, leaving pan mode"
	
		if event.button()==Qt.Qt.RightButton:
			self.setDragMode(QtGui.QGraphicsView.NoDrag)
			
			print "Right mouse button released, leaving pan mode"
			
		if event.button()==Qt.Qt.MidButton:
			self.setDragMode(QtGui.QGraphicsView.NoDrag)
			
			print "Middle mouse button released, leaving pan mode"
			
		QtGui.QGraphicsView.mouseReleaseEvent(self,event)  

##
#This is the inherited GraphicsScene on which The items are drawn
##

class ComponentScene(QtGui.QGraphicsScene):#The scene onwhich we are drawing the graph
	def __init__(self,parent=None):
		QtGui.QGraphicsScene.__init__(self)
		self.parent=parent

##
#This is the Buttons shown on the right side of the tool..
##
			
class DirectoryItem(QtGui.QPushButton):#This will be listed on the right most side of the software
	def __init__(self,parent=None,args=None,name="Component"):
		QtGui.QPushButton.__init__(self,args)
		self.parent=parent
		self.args=args
		self.connect(self,QtCore.SIGNAL("clicked()"),self.clickEvent)
		QtGui.QPushButton.setIcon(self,QtGui.QIcon(QtGui.QPixmap(getDefaultIconPath())))
		self.setText(name)

	def setIcon(self,arg):
		self.Icon=QtGui.QIcon()
		self.Icon.addPixmap(arg)
		QtGui.QPushButton.setIcon(self,self.Icon)

	def contextMenuEvent(self,event):
		self.parent.View.CompoPopUpMenu.setComponent(self.parent.graphicsItem)
		self.parent.View.CompoPopUpMenu.popup(event.globalPos())

	def clickEvent(self):#What happens when clicked
		#print "Clicked"+ self.parent.alias
		index=self.parent.mainWindow.UI.tabWidget.currentIndex()
		#print index
		self.parent.mainWindow.currentComponent=self.parent
		if index==0:
			self.parent.CommonProxy.setVisibility(True)
			#print "Set visiblitiy true"
		if index==1:
			self.parent.mainWindow.CodeEditor.findFirst(self.parent.alias,False,True,True,True)

##
#This classes will take care of multiplying the position.That is if the nodes are too close to each other they will strech them
##
class PositionMultiplier_Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.setWindowModality(QtCore.Qt.WindowModal)
        Dialog.resize(415, 119)
        self.gridLayout = QtGui.QGridLayout(Dialog)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_2 = QtGui.QLabel(Dialog)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_3.addWidget(self.label_2)
        self.doubleSpinBox_2 = QtGui.QDoubleSpinBox(Dialog)
        self.doubleSpinBox_2.setProperty("value", 1.0)
        self.doubleSpinBox_2.setObjectName(_fromUtf8("doubleSpinBox_2"))
        self.horizontalLayout_3.addWidget(self.doubleSpinBox_2)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label = QtGui.QLabel(Dialog)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout.addWidget(self.label)
        self.doubleSpinBox = QtGui.QDoubleSpinBox(Dialog)
        self.doubleSpinBox.setProperty("value", 1.0)
        self.doubleSpinBox.setObjectName(_fromUtf8("doubleSpinBox"))
        self.horizontalLayout.addWidget(self.doubleSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_5.addItem(spacerItem)
        self.pushButton_2 = QtGui.QPushButton(Dialog)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_5.addWidget(self.pushButton_2)
        self.pushButton = QtGui.QPushButton(Dialog)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout_5.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(_translate("Dialog", "Position Multiplier", None))
        self.label_2.setText(_translate("Dialog", "XStretch", None))
        self.label.setText(_translate("Dialog", "YStretch", None))
        self.pushButton_2.setText(_translate("Dialog", "Apply", None))
        self.pushButton.setText(_translate("Dialog", "OK", None))


class PositionMultiplier(QtGui.QDialog):
	def __init__(self,logger):
		QtGui.QDialog.__init__(self)
		self.logger=logger
		self.UI=PositionMultiplier_Ui_Dialog()
		self.UI.setupUi(self)
		self.currentXstretch=1
		self.currentYstretch=1
		self.compList=[]
		self.settings=None
		self.ChangePermanently=False
		self.connect(self.UI.pushButton,QtCore.SIGNAL("clicked()"),self.setPermanent)
		self.connect(self.UI.pushButton_2,QtCore.SIGNAL("clicked()"),self.setTemporary)
		
	def updateStretch(self,compList,settings):
		self.show()
		self.compList=compList
		self.settings
		self.UI.doubleSpinBox.setValue(1)
		self.UI.doubleSpinBox_2.setValue(1)
		self.ChangePermanently=False
		for x in self.compList:
			x.tempx=x.x
			x.temy=x.y
			
	def setTemporary(self):
		for x in self.compList:
			x.tempx=x.x*self.UI.doubleSpinBox_2.value()
			x.tempy=x.y*self.UI.doubleSpinBox.value()
		for x in self.compList:
			x.graphicsItem.setPos(QtCore.QPointF(x.tempx,x.tempy))
			x.graphicsItem.updateforDrag()
			
	def setPermanent(self):
		self.ChangePermanently=True
		self.close()
		for x in self.compList:
			x.x=x.x*self.UI.doubleSpinBox_2.value()
			x.y=x.y*self.UI.doubleSpinBox.value()
		for x in self.compList:
			x.graphicsItem.setPos(QtCore.QPointF(x.x,x.y))
			x.graphicsItem.updateforDrag()
			
	def closeEvent(self,event):		
		if self.ChangePermanently==True:
			self.logger.logData("Graph Stretched")
		else:
			for x in self.compList:
				x.graphicsItem.setPos(QtCore.QPointF(x.x,x.y))
				x.graphicsItem.updateforDrag()
				
##
#This class will communicate with the common behavior component..On construction
##

class commonBehaviorComponent(threading.Thread):
	def __init__(self,parent):
		threading.Thread.__init__(self)
		self.initialize()
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.daemon = True
		self.proxy=None
		self.parent=parent
		self.Frequency=0
		self.TimeAwake=0
		self.visible=False
		self.refreshTime=.5
		self.work=True
		self.start()
		
	def initialize(self):
		pass
		
	def run(self):
		#global global_ic
		#print "started running"
		while self.work == True:
			if self.visible==True:
				pass #Define here what ever have to be done during the first tab is displayed
			time.sleep(self.refreshTime)

	def setRefreshTime(self,time):#To be called to set how often the variables should be reset.
		if time>10:
			self.refreshTime=time
		else:
			self.refreshTime=10

	def setVisibility(self,status=True):
		self.mutex.lock()
		self.visible = status
		#print "visible change"+str(status)
		self.mutex.unlock()

##
#This is the menu which will appear when right click on a component 
##
	
class ComponentMenu(QtGui.QMenu):
	def  __init__(self,parent):
		QtGui.QMenu.__init__(self,parent)
		
		self.ActionUp=QtGui.QAction("Up",parent)
		self.ActionDown=QtGui.QAction("Down",parent)
		self.ActionEdit=QtGui.QAction("Edit",parent)
		self.ActionControl=QtGui.QAction("Control",parent)
		self.ActionNewConnection=QtGui.QAction("New Connection",parent)
		self.ActionAddToGroup=QtGui.QAction("Add to Group",parent)
		self.ActionDelete=QtGui.QAction("Delete",parent)
		self.ActionRemoveFromGroup=QtGui.QAction("Remove Group",parent)
		self.ActionUpGroup=QtGui.QAction("UP All",parent)
		self.ActionDownGroup=QtGui.QAction("DOWN All",parent)
		#self.ActionFreq=QtGui.QAction("Freq",parent)

		self.GroupMenu=QtGui.QMenu("Group",parent)
		self.GroupMenu.addAction(self.ActionAddToGroup)
		self.GroupMenu.addAction(self.ActionRemoveFromGroup)
		self.GroupMenu.addAction(self.ActionUpGroup)
		self.GroupMenu.addAction(self.ActionDownGroup)

		#self.addAction(self.ActionFreq)
		self.addAction(self.ActionDelete)
		self.addAction(self.ActionUp)
		self.addAction(self.ActionDown)
		self.addAction(self.ActionNewConnection)
		self.addMenu(self.GroupMenu)
		self.addAction(self.ActionControl)
		self.addAction(self.ActionEdit)

	def setComponent(self,component):
		self.currentComponent=component

##
#This is the menu which appears when right click on the background of the graphics Scene
##

class BackgroundMenu(QtGui.QMenu):
	def __init__(self,parent):
		QtGui.QMenu.__init__(self,parent)
		self.ActionSettings=QtGui.QAction("Settings",parent)
		self.ActionUp=QtGui.QAction("Up All",parent)
		self.ActionDown=QtGui.QAction("Down All",parent)
		self.ActionSearch=QtGui.QAction("Search",parent)
		self.ActionAdd=QtGui.QAction("Add Component",parent)
		self.ActionNewGroup=QtGui.QAction("New Group",parent)
		self.ActionStretch=QtGui.QAction("Stretch",parent)
		
		self.GraphMenu=QtGui.QMenu("Graph",parent)
		self.GraphMenu.addAction(self.ActionStretch)
		
		self.addMenu(self.GraphMenu)
		self.addAction(self.ActionUp)
		self.addAction(self.ActionDown)
		self.addAction(self.ActionAdd)
		self.addAction(self.ActionNewGroup)
		self.addAction(self.ActionSettings)
		self.addAction(self.ActionSearch)
		self.pos=None
		
	def setPos(self,pos):
		self.pos=pos

##
#This is the container class to contain values about the network..It also contains the network groups 
##
#

class NetworkValues():##This will contain Values about the network
	def __init__(self):
		self.xstrech=1
		self.ystrech=1
		self.spring_length=100
		self.field_force_multiplier=20000
		self.hookes_constant=.07
		self.time_elapsed2=3
		self.roza=.8
		self.Groups=[]

##
#Below are the functions regarding reading from file writing to file and stuffs like that.
##

def getStringFromFile(path):##This is the first function to be called for reading configurations for a xml file
	try:
		file = open(path, 'r')
	except:
		print 'Can\'t open ' + path + '.'

	data = file.read()
	return data
	
def getDataFromString(data,logger):#Data is a string in xml format containing information	
	components = []	
	try:
		xmldoc = libxml2.parseDoc(data)
		root = xmldoc.children
	except Exception,e:
		logger.logData(str(e),"R")
	
	if root is not None:
		components, NetworkSettings = parsercmanager(root,logger)
	xmldoc.freeDoc()
	return components, NetworkSettings

def parsercmanager(node,logger): #Call seperate functions for general settings and nodes
	components = []
	generalSettings=NetworkValues()
	#print "Inside parsercmanager"
	if node.type == "element" and node.name == "rcmanager":
		child = node.children
		while child is not None:
			if child.type == "element":
				if child.name == "generalInformation":
					parseGeneralInformation(child, generalSettings,logger)
				elif child.name == "node":
					#print "parseing node"
					parseNode(child,components,generalSettings, logger)
				elif stringIsUseful(str(node.properties)):
					print 'ERROR when parsing rcmanager: '+str(child.name)+': '+str(child.properties)
			child = child.next
	return components, generalSettings

def parseGeneralInformation(node, generalSettings,logger): ##Takes care of reading the general information about the network tree
	#print "Entered the parseGeneralInformation"
	if node.type == "element" and node.name == "generalInformation":
		child =node.children
		while child is not None:
			if child.type=="element":
				if child.name=="group":
			#		print "Started reading a group"
					try:
						group=ComponentGroup()
						group.setName(parseSingleValue(child,"name"))
						group.setIconFilePath(parseSingleValue(child,"iconfile"))
						group.readFromIconFile()
						generalSettings.Groups.append(group)
					except Exception,e:
						logger.logData("Couldn Read group "+str(e),"R")
					else:
						logger.logData("Read the group ::"+ group.groupName +" Information")
				elif child.name=="simulation":
					generalSettings.spring_length=float(parseSingleValue(child,"springlength"))
					generalSettings.field_force_multiplier=float(parseSingleValue(child,"fieldforce"))
					generalSettings.hookes_constant=float(parseSingleValue(child,"hookes"))
					generalSettings.roza=1- float(parseSingleValue(child,"friction"))
				
			child=child.next

		logger.logData("General Information Successfully read")

def checkForMoreProperties(node):
	if node.properties != None: print 'WARNING: Attributes unexpected: ' + str(node.properties)

def searchForGroupName(settings,name):
	flag=0
	for x in  settings.Groups:
		if x.groupName==name:
			flag=1
			return x
	if flag==0:
		raise Exception("Couldn't find a group with name"+name)

def parseNode(node, components,generalSettings,logger):#To get the properties of a component
	if node.type == "element" and node.name == "node":
		child = node.children
		comp = CompInfo()
		#comp.CheckItem.setLogger(logger)
		comp.alias = parseSingleValue(node, 'alias', False)
		#print "Started reading component:: "+comp.alias
		comp.DirectoryItem.setText(comp.alias)
		comp.endpoint = parseSingleValue(node, 'endpoint', False)
		while child is not None:
			if child.type == "element":
				if child.name=="group":
					comp.groupName=parseSingleValue(child,"name")
					try:
						group=searchForGroupName(generalSettings,comp.groupName)
						group.addComponent(comp)
					except Exception,e:
						logger.logData("Couldnt add Component to the group::"+comp.groupName+" "+str(e),"R")	
				
				elif child.name == "workingDir":
					comp.workingdir = parseSingleValue(child, 'path')
				elif child.name == "upCommand":
					comp.compup = parseSingleValue(child, 'command')
				elif child.name == "downCommand":
					comp.compdown = parseSingleValue(child, 'command')
				elif child.name == "configFile":
					comp.configFile = parseSingleValue(child, 'path')
				elif child.name=="radius":##To be backward compatible with older tool
					pass
				elif child.name == "xpos":
					x=parseSingleValue(child, 'value')
					try :
						comp.x=float(x)
					except :
						logger.logData("Error in Reading Position Value of "+comp.alias,"R")
				elif child.name == "ypos":
					y = float(parseSingleValue(child, 'value'))					
					try :
						comp.y=float(y)
					except :
						logger.logData("Error in Reading Position Value of "+comp.alias,"R")
				elif child.name=="color":
					comp.nodeColor=getColorFromString(parseSingleValue(child,"value"))
				elif child.name == "dependence":
					comp.dependences.append(parseSingleValue(child, 'alias'))
				elif child.name == "ip":
					comp.Ip=parseSingleValue(child, "value")
				elif stringIsUseful(str(child.properties)):
					print 'ERROR when parsing rcmanager: '+str(child.name)+': '+str(child.properties)
			child = child.next
		
		checkForCompChildren(comp,node)##This function si there to verify All neccessary details are added 
		components.append(comp)

	elif node.type == "text":
		if stringIsUseful(str(node.properties)):
			print '    tssssssss'+str(node.properties)
	else:
		print "error: "+str(node.name)
	
	comp.CheckItem.start()
	
def getColorFromString(string):
	#print string
	color=[]
	color.append(int(string.__getslice__(1,3),16))
	color.append(int(string.__getslice__(3,5),16))
	color.append(int(string.__getslice__(5,7),16))
	#print color
	return color
	
def stringIsUseful(string):
	if len(string) == 0: return False
	if string[0] == '#': return False

	s1 = string
	s1 = s1.replace(' ', '')
	s1 = s1.replace('\t', '')
	s1 = s1.replace('\n', '')

	if len(s1) == 0: return False
	return True

def parseSingleValue(node, arg, doCheck=True, optional=False):
	if node.children != None and doCheck == True: print 'WARNING: No children expected'+str(node)
	
	if not node.hasProp(arg) and not optional:
		print 'WARNING: ' + arg + ' attribute expected'
	else:
		ret = node.prop(arg)
		node.unsetProp(arg)
		return ret

def getXmlFromNetwork(NetworkSettings, components,logger):
	string=''
	string=string+'<?xml version="1.0" encoding="UTF-8"?>\n'
	string=string+'<rcmanager>\n\n'
	string=string+'\t<generalInformation>\n'

	for x in NetworkSettings.Groups:
		string=string+'\t\t<group name="'+x.groupName+'" iconfile="'+x.groupIconFilePath+'" />\n'

	string=string+'\t\t<simulation hookes="'+str(NetworkSettings.hookes_constant)+'" friction="'+str(1-NetworkSettings.roza)+'" springlength="'+str(NetworkSettings.spring_length)+'" fieldforce="'+str(NetworkSettings.field_force_multiplier)+'"/>\n'
	string=string+'\n\t</generalInformation>\n'
	
	for comp in components:
		comp.x=comp.graphicsItem.x()
		comp.y=comp.graphicsItem.y()
		string=string+'\n\t<node alias="' + comp.alias + '" endpoint="' + comp.endpoint + '">\n'
		for dep in comp.asBeg:
			string=string+'\t\t<dependence alias="' + dep.toComponent.alias + '" />\n'
		if comp.groupName!="":
			string=string+'\t\t<group name="' + comp.groupName+ '" />\n'	
		string=string+'\t\t<workingDir path="' + comp.workingdir + '" />\n'
		string=string+'\t\t<upCommand command="' + comp.compup + '" />\n'
		string=string+'\t\t<downCommand command="' + comp.compdown + '" />\n'
		string=string+'\t\t<configFile path="' + comp.configFile + '" />\n'
		string=string+'\t\t<xpos value="' + str(comp.x) + '" />\n'
		string=string+'\t\t<ypos value="' + str(comp.y) + '" />\n'
		#string=string+'\t\t<icon value="'+str(comp.IconFilePath)+'"/>\n'
		string=string+'\t\t<ip value="'+str(comp.Ip)+'"/>\n'
		string=string+'\t</node>\n'

	string=string+'\n</rcmanager>'
	return string

def writeToFile(file, string):#Write a line to the file
	file.write((string).encode( "utf-8" ))

def getDefaultNode():
	string="\n"
	string=string+ '\t<node alias=" " endpoint=" ">\n'
	string=string+ '\t\t<dependence alias=" " />\n'
	string=string+ '\t\t<workingDir path=" " />\n'
	string=string+ '\t\t<upCommand command=" " />\n'
	string=string+ '\t\t<downCommand command=" " />\n'
	string=string+ '\t\t<configFile path=" " />\n'
	string=string+ '\t\t<xpos value=" " />\n'
	string=string+ '\t\t<ypos value=" " />\n'
	string=string+ '\t\t<icon value=" "/>\n'
	string=string+ '\t\t<ip value=" "/>\n'
	string=string+ '\t</node>\n'
	return string

def getDefaultSettings():
	string="\n"
	string=string+'\t<generalInformation>\n'
	string=string+'\t</generalInformation>\n'
	return string

def checkForCompChildren(comp,Xmlnode):#This will check whether the opened Doc have enough children to draw the things
	return

def searchForChild(Xmlnode,Name):##Will search in tree for node with name Name
	child=Xmlnode.children
	while child is not None:
		if child.type=="element" and child.name==Name:
			return True
		child=child.next

	return False

def getDefaultIconPath():
	return "/opt/robocomp/share/rcmanager/1465594390_sign-add.png" #This is the default icon can be changed by users choice

def upComponent(component,Logger):#Just Up the component
		try:
			if component.CheckItem.haveStarted()==False:
				component.CheckItem.initializeComponent()
			if component.CheckItem.haveStarted()==False:
				Logger.logData("Component "+component.alias+" Cannot be Monitored because of bad Proxy setting(Error ignored)","R")	
			proc=QtCore.QProcess()
			proc.startDetached(component.compup)
			
		except Exception, e:
			Logger.logData("Cannot write"+str(e),"R")
			raise e
		else:
			pass
		finally:
			pass	

def downComponent(component,Logger):#To down a particular component
		try:
			proc=QtCore.QProcess()
			proc.startDetached(component.compdown)
		except Exception, e:
			Logger.logData("Cannot Kill"+str(e),"R")
			raise e
		else:
			pass
		finally:
			pass

def getXmlNode(editor,name):
	flag=False
	while flag==False:
		editor.findFirst(name,False,True,True,True)
		CursPoint=editor.getCursorPosition()
