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
import logging
from PyQt4 import QtGui, QtCore

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

class LogFileSetterDialog(QtGui.QDialog):
    def __init__(self, parent):
        QtGui.QDialog.__init__(self)
        self.parent = parent
        self._logger = logging.getLogger('RCManager.LogFileSetterDialog')
        self._logger.setLevel(logging.DEBUG)
        self.setupUi()
        self.connect(self.pushButton, QtCore.SIGNAL("clicked()"), self.browse)
        self.connect(self.pushButton_2, QtCore.SIGNAL("clicked()"), self.ok)

    def setupUi(self):
        self.setObjectName(_fromUtf8("Dialog"))
        self.resize(453, 84)
        self.gridLayout = QtGui.QGridLayout(self)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.lineEdit = QtGui.QLineEdit(self)
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))
        self.horizontalLayout.addWidget(self.lineEdit)
        self.pushButton = QtGui.QPushButton(self)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem)
        self.pushButton_2 = QtGui.QPushButton(self)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_2.addWidget(self.pushButton_2)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        self.setWindowTitle(_translate("Dialog", "Log File", None))
        self.pushButton.setText(_translate("Dialog", "Browse", None))
        self.pushButton_2.setText(_translate("Dialog", "Ok", None))

    def setFile(self):
        self.show()
        self.lineEdit.setText(self.logger.filename)

    def browse(self):
        filename = QtGui.QFileDialog.getSaveFileName(self, 'Save File', os.getcwd(), '*.log')
        filename = str(filename)
        if filename.endswith(".log") is False:
            filename = filename + ".log"
        self.lineEdit.setText(filename)

    def ok(self):
        self.logger.setFile(str(self.lineEdit.text()))
        self.close()


class GroupBuilderDialog(QtGui.QDialog):
    def __init__(self, parent):
        QtGui.QDialog.__init__(self)
        self.parent = parent
        self._logger = logging.getLogger('RCManager.GroupBuilderDialog')
        self._logger.setLevel(logging.DEBUG)
        self.setupUi()
        self.connect(self.pushButton_3, QtCore.SIGNAL("clicked()"), self.SaveGroup)
        self.connect(self.pushButton_2, QtCore.SIGNAL("clicked()"), self.cancel)
        self.connect(self.pushButton, QtCore.SIGNAL("clicked()"), self.browseIcon)
        self.group = None
        self.build = False

    def setupUi(self):
        self.setObjectName(_fromUtf8("Dialog"))
        self.resize(543, 135)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sizePolicy().hasHeightForWidth())
        self.setSizePolicy(sizePolicy)
        self.gridLayout = QtGui.QGridLayout(self)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label = QtGui.QLabel(self)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout.addWidget(self.label)
        self.lineEdit = QtGui.QLineEdit(self)
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))
        self.horizontalLayout.addWidget(self.lineEdit)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_2 = QtGui.QLabel(self)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_3.addWidget(self.label_2)
        self.lineEdit_2 = QtGui.QLineEdit(self)
        self.lineEdit_2.setObjectName(_fromUtf8("lineEdit_2"))
        self.horizontalLayout_3.addWidget(self.lineEdit_2)
        self.pushButton = QtGui.QPushButton(self)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout_3.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem)
        self.pushButton_2 = QtGui.QPushButton(self)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_4.addWidget(self.pushButton_2)
        self.pushButton_3 = QtGui.QPushButton(self)
        self.pushButton_3.setAutoDefault(False)
        self.pushButton_3.setDefault(True)
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))
        self.horizontalLayout_4.addWidget(self.pushButton_3)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        self.setWindowTitle(_translate("self", "Group Builder", None))
        self.label.setText(_translate("Dialog", "Group Name", None))
        self.label_2.setText(_translate("Dialog", "Icon File         ", None))
        self.pushButton.setText(_translate("Dialog", "Browse", None))
        self.pushButton_2.setText(_translate("Dialog", "Cancel", None))
        self.pushButton_3.setText(_translate("Dialog", "Ok", None))

    def startBuildGroup(self, networkSettings):
        self.build = False
        self.networkSettings = networkSettings
        self.group = ComponentGroup()
        self.show()

    def SaveGroup(self):
        self.group.setName(self.lineEdit.text())
        self.group.set_icon_file_path(self.lineEdit_2.text())
        self.group.read_from_icon_file()
        self.networkSettings.Groups.append(self.group)
        self.parent.refresh_code_from_tree()
        self._logger.info("New Group::" + self.lineEdit.text() + " Build")
        self.lineEdit.setText("")
        self.lineEdit_2.setText("")
        self.build = True
        self.close()

    def cancel(self):
        self.build = False
        self.close()

    def browseIcon(self):
        self.lineEdit_2.setText(
            QtGui.QFileDialog.getOpenFileName(self, 'Set Group Icon', os.getcwd(), "Image Files (*.png *.jpg *.bmp)"))

    def closeEvent(self, event):
        QtGui.QDialog.closeEvent(self, event)
        if self.build == False:
            self._logger.info("Group Building Canceled by User", "R")
            self.lineEdit.setText("")
            self.lineEdit_2.setText("")
        self.build = False


class GroupSelectorDialog(QtGui.QDialog):
    def __init__(self, parent):
        self._logger = logging.getLogger('RCManager.GroupSelectorDialog')
        self._logger.setLevel(logging.DEBUG)
        self.parent = parent
        QtGui.QDialog.__init__(self)
        self.setupUi()
        self.connect(self.pushButton_2, QtCore.SIGNAL("clicked()"), self.cancel)
        self.connect(self.pushButton, QtCore.SIGNAL("clicked()"), self.selected)
        self.groupList = None
        self.component = None
        self.groupAdded = False

    def setupUi(self):
        self.setObjectName(_fromUtf8("Dialog"))
        self.resize(282, 394)
        self.gridLayout = QtGui.QGridLayout(self)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.listWidget = QtGui.QListWidget(self)
        self.listWidget.setObjectName(_fromUtf8("listWidget"))
        self.horizontalLayout.addWidget(self.listWidget)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.pushButton_2 = QtGui.QPushButton(self)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_2.addWidget(self.pushButton_2)
        self.pushButton = QtGui.QPushButton(self)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout_2.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        self.setWindowTitle(_translate("Dialog", "Group Selector", None))
        self.pushButton_2.setText(_translate("Dialog", "Cancel", None))
        self.pushButton.setText(_translate("Dialog", "Ok", None))

    def openSelector(self, component, groupList):
        self.groupAdded = False
        self.groupList = groupList
        self.component = component
        self.listWidget.clear()
        for x in self.groupList:
            item = QtGui.QListWidgetItem(QtGui.QIcon(x.groupIconPixmap), x.groupName)
            self.listWidget.addItem(item)
        self.show()
        self.compoent = component
        self.groupList = groupList

    def cancel(self):
        self.groupAdded = False
        self.close()

    def selected(self):
        string = self.listWidget.currentItem().text()
        for x in self.groupList:
            if x.groupName == string:
                x.add_component(self.component)
        self.listWidget.clear()
        self.groupAdded = True
        self._logger.info("Component ::" + self.component.alias + " Added to group " + self.component.groupName)
        self.parent.refresh_code_from_tree()
        self.close()

    def closeEvent(self, event):
        if self.groupAdded == False:
            QtGui.QDialog.closeEvent(self, event)
            self._logger.info("Adding to group Cancelled by User", "R")
            self.listWidget.clear()

class ConnectionBuilderDialog(QtGui.QDialog):  ## This is used to set connection between two different dialogs
    def __init__(self, parent):
        QtGui.QDialog.__init__(self)

        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
        self._logger = logging.getLogger('RCManager.ConnectionBuilderDialog')
        self._logger.setLevel(logging.DEBUG)
        self.parent = parent
        self.setupUi()
        self.connection = None
        self.fromComponent = None
        self.toComponent = None
        self.BuildingStatus = False
        self.connect(self.pushButton, QtCore.SIGNAL("clicked()"), self.SaveConnection)
        self.connect(self.pushButton_2, QtCore.SIGNAL("clicked()"), self.closeWithoutSaving)

    def setupUi(self):
        self.setObjectName(_fromUtf8("Dialog"))
        self.resize(434, 138)
        self.gridLayout = QtGui.QGridLayout(self)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_2 = QtGui.QLabel(self)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_2.addWidget(self.label_2)
        self.lineEdit_2 = QtGui.QLineEdit(self)
        self.lineEdit_2.setObjectName(_fromUtf8("lineEdit_2"))
        self.horizontalLayout_2.addWidget(self.lineEdit_2)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label = QtGui.QLabel(self)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout.addWidget(self.label)
        self.lineEdit = QtGui.QLineEdit(self)
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))
        self.horizontalLayout.addWidget(self.lineEdit)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem)
        self.pushButton_2 = QtGui.QPushButton(self)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_3.addWidget(self.pushButton_2)
        self.pushButton = QtGui.QPushButton(self)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout_3.addWidget(self.pushButton)
        self.gridLayout.addLayout(self.horizontalLayout_3, 1, 0, 1, 1)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        self.setWindowTitle(_translate("Dialog", "Connection Builder", None))
        self.label_2.setText(_translate("Dialog", "From", None))
        self.label.setText(_translate("Dialog", "To", None))
        self.pushButton_2.setText(_translate("Dialog", "Cancel", None))
        self.pushButton.setText(_translate("Dialog", "Ok", None))

    def buildNewConnection(self):
        self.connection = NodeConnection()
        self.BuildingStatus = True

    def setBeg(self, component):
        self.lineEdit_2.setText(component.alias)
        self.connection.fromComponent = component
        self.fromComponent = component

    def setEnd(self, component):
        self.lineEdit.setText(component.alias)
        self.connection.toComponent = component
        self.toComponent = component

    def SaveConnection(self):
        self.toComponent.dependences.append(self.fromComponent.alias)
        self.toComponent.asEnd.append(self.connection)
        self.fromComponent.asBeg.append(self.connection)
        self.parent.NetworkScene.addItem(self.connection)
        self.close()
        self.parent.NetworkScene.update()
        self.parent.refresh_code_from_tree()
        self._logger.info("Connection Made From " + self.fromComponent.alias + " to " + self.toComponent.alias)
        self.BuildingStatus = False
        self.lineEdit.setText("")
        self.lineEdit_2.setText("")

    def closeWithoutSaving(self):
        self.close()
        self.lineEdit.setText("")
        self.lineEdit_2.setText("")


##
# This will takes care of selecting the rcmanger tool settings..The first class is about UI.second is main thing..
##

class NetworkSettingsDialog(QtGui.QDialog):  # This will show a dialog window for selecting the rcmanager tool settings
    def __init__(self, parent=None):
        QtGui.QDialog.__init__(self)
        self.parent = parent
        self.setupUi()
        self.setting = None

    def setupUi(self):
        self.setObjectName(_fromUtf8("Dialog"))
        self.resize(561, 263)
        self.gridLayout = QtGui.QGridLayout(self)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem)
        self.pushButton_2 = QtGui.QPushButton(self)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_4.addWidget(self.pushButton_2)
        self.pushButton_3 = QtGui.QPushButton(self)
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))
        self.horizontalLayout_4.addWidget(self.pushButton_3)
        self.pushButton = QtGui.QPushButton(self)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout_4.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.gridLayout.addLayout(self.verticalLayout, 1, 0, 1, 1)
        self.formLayout = QtGui.QFormLayout()
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setObjectName(_fromUtf8("formLayout"))
        self.label = QtGui.QLabel(self)
        self.label.setObjectName(_fromUtf8("label"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.LabelRole, self.label)
        self.doubleSpinBox = QtGui.QDoubleSpinBox(self)
        self.doubleSpinBox.setObjectName(_fromUtf8("doubleSpinBox"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.doubleSpinBox)
        self.label_2 = QtGui.QLabel(self)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label_2)
        self.doubleSpinBox_2 = QtGui.QDoubleSpinBox(self)
        self.doubleSpinBox_2.setObjectName(_fromUtf8("doubleSpinBox_2"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.doubleSpinBox_2)
        self.label_3 = QtGui.QLabel(self)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label_3)
        self.doubleSpinBox_3 = QtGui.QDoubleSpinBox(self)
        self.doubleSpinBox_3.setObjectName(_fromUtf8("doubleSpinBox_3"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.doubleSpinBox_3)
        self.label_4 = QtGui.QLabel(self)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.formLayout.setWidget(3, QtGui.QFormLayout.LabelRole, self.label_4)
        self.doubleSpinBox_4 = QtGui.QDoubleSpinBox(self)
        self.doubleSpinBox_4.setObjectName(_fromUtf8("doubleSpinBox_4"))
        self.formLayout.setWidget(3, QtGui.QFormLayout.FieldRole, self.doubleSpinBox_4)
        spacerItem1 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.formLayout.setItem(4, QtGui.QFormLayout.FieldRole, spacerItem1)
        self.gridLayout.addLayout(self.formLayout, 0, 0, 1, 1)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        self.setWindowTitle(_translate("Dialog", "NetworkSettings", None))
        self.pushButton_2.setText(_translate("Dialog", "Apply", None))
        self.pushButton_3.setText(_translate("Dialog", "Cancel", None))
        self.pushButton.setText(_translate("Dialog", "OK", None))
        self.label.setText(_translate("Dialog", "Connection Modularity", None))
        self.label_2.setText(_translate("Dialog", "Node Modularity", None))
        self.label_3.setText(_translate("Dialog", "X-Multiplication factor", None))
        self.label_4.setText(_translate("Dialog", "Y-Multiplication factor", None))


    def setData(self, setting):
        self.setting = setting


##
# This will take care of the Save warning and stuff to avoid accidental quiting without saving ..First class is for UI..Second one takes care of the main stuffs..
##


class SaveWarningDialog(QtGui.QDialog):  # To be used as a warning window while deleting existing tree without saving
    def __init__(self, parent=None):
        QtGui.QDialog.__init__(self)
        self.parent = parent
        self.setupUi()
        self.connect(self.pushButton_2, QtCore.SIGNAL("clicked()"), self.save)
        self.connect(self.pushButton, QtCore.SIGNAL("clicked()"), self.dontSave)
        self.connect(self.pushButton_3, QtCore.SIGNAL("clicked()"), self.cancel)
        self.setModal(True)
        self.Status = "C"

    def setupUi(self):
        self.setObjectName(_fromUtf8("Dialog"))
        self.resize(493, 87)
        self.horizontalLayout = QtGui.QHBoxLayout(self)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.pushButton = QtGui.QPushButton(self)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtGui.QPushButton(self)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8("share/rcmanager/1465394415_floppy.png")), QtGui.QIcon.Normal,
                       QtGui.QIcon.Off)
        self.pushButton_2.setIcon(icon)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout.addWidget(self.pushButton_2)
        self.pushButton_3 = QtGui.QPushButton(self)
        self.pushButton_3.setAutoDefault(True)
        self.pushButton_3.setDefault(True)
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))
        self.horizontalLayout.addWidget(self.pushButton_3)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        self.setWindowTitle(_translate("Dialog", "Save Warning", None))
        self.pushButton.setText(_translate("Dialog", "Don\'t Save", None))
        self.pushButton_2.setText(_translate("Dialog", "Save", None))
        self.pushButton_3.setText(_translate("Dialog", "Cancel", None))

    def decide(self):
        self.exec_()
        return self.Status

    def save(self):
        self.close()
        self.Status = "S"

    def dontSave(self):
        self.close()
        self.Status = "D"

    def cancel(self):
        self.close()
        self.Status = "C"

    ##
    # This classes will take care of multiplying the position.That is if the nodes are too close to each other they will strech them
    ##

class PositionMultiplierDialog(QtGui.QDialog):
    def __init__(self):
        QtGui.QDialog.__init__(self)
        self._logger = logging.getLogger('RCManager.PositionMultiplierDialog')
        self._logger.setLevel(logging.DEBUG)
        self.setupUi()
        self.currentXstretch = 1
        self.currentYstretch = 1
        self.compList = []
        self.settings = None
        self.ChangePermanently = False
        self.connect(self.pushButton, QtCore.SIGNAL("clicked()"), self.setPermanent)
        self.connect(self.pushButton_2, QtCore.SIGNAL("clicked()"), self.setTemporary)

    def setupUi(self):
        self.setObjectName(_fromUtf8("Dialog"))
        self.setWindowModality(QtCore.Qt.WindowModal)
        self.resize(415, 119)
        self.gridLayout = QtGui.QGridLayout(self)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_2 = QtGui.QLabel(self)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_3.addWidget(self.label_2)
        self.doubleSpinBox_2 = QtGui.QDoubleSpinBox(self)
        self.doubleSpinBox_2.setProperty("value", 1.0)
        self.doubleSpinBox_2.setObjectName(_fromUtf8("doubleSpinBox_2"))
        self.horizontalLayout_3.addWidget(self.doubleSpinBox_2)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label = QtGui.QLabel(self)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout.addWidget(self.label)
        self.doubleSpinBox = QtGui.QDoubleSpinBox(self)
        self.doubleSpinBox.setProperty("value", 1.0)
        self.doubleSpinBox.setObjectName(_fromUtf8("doubleSpinBox"))
        self.horizontalLayout.addWidget(self.doubleSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_5.addItem(spacerItem)
        self.pushButton_2 = QtGui.QPushButton(self)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_5.addWidget(self.pushButton_2)
        self.pushButton = QtGui.QPushButton(self)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout_5.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        self.setWindowTitle(_translate("Dialog", "Position Multiplier", None))
        self.label_2.setText(_translate("Dialog", "XStretch", None))
        self.label.setText(_translate("Dialog", "YStretch", None))
        self.pushButton_2.setText(_translate("Dialog", "Apply", None))
        self.pushButton.setText(_translate("Dialog", "OK", None))

    def updateStretch(self, compList, settings):
        self.show()
        self.compList = compList
        self.settings
        self.doubleSpinBox.setValue(1)
        self.doubleSpinBox_2.setValue(1)
        self.ChangePermanently = False
        for x in self.compList:
            x.tempx = x.x
            x.temy = x.y

    def setTemporary(self):
        for x in self.compList:
            x.tempx = x.x * self.doubleSpinBox_2.value()
            x.tempy = x.y * self.doubleSpinBox.value()
        for x in self.compList:
            x.graphicsItem.setPos(QtCore.QPointF(x.tempx, x.tempy))
            x.graphicsItem.updateforDrag()

    def setPermanent(self):
        self.ChangePermanently = True
        self.close()
        for x in self.compList:
            x.x = x.x * self.doubleSpinBox_2.value()
            x.y = x.y * self.doubleSpinBox.value()
        for x in self.compList:
            x.graphicsItem.setPos(QtCore.QPointF(x.x, x.y))
            x.graphicsItem.updateforDrag()

    def closeEvent(self, event):
        if self.ChangePermanently == True:
            self._logger.info("Graph Stretched")
        else:
            for x in self.compList:
                x.graphicsItem.setPos(QtCore.QPointF(x.x, x.y))
                x.graphicsItem.updateforDrag()


if __name__ == '__main__':
    import sys
    from PyQt4.QtGui import QApplication
    import inspect
    from random import randint
    # process params with a argparse
    app = QApplication(sys.argv)

    # Load all classes defined in this module
    dialogs = {}
    class_members = inspect.getmembers(sys.modules[__name__], inspect.isclass)
    for cls in class_members:
        # Loop over all classes but QApplication
        if 'QApplication' not in cls[0]:
            print "Found class %s" % cls[0]
            # Get params for init of the class and initalize to None
            params = inspect.getargspec(cls[1].__init__)
            args = {}
            for attr in params[0]:
                if attr and not 'self' in attr:
                    args[attr] = None
            dialogs[cls[0]] = cls[1](*args)
            x_pos = randint(0, 1280)
            y_pos = randint(0, 1024)
            dialogs[cls[0]].move(x_pos, y_pos)
            # Show the Dialog
            dialogs[cls[0]].show()
    ret = app.exec_()
    sys.exit(ret)


