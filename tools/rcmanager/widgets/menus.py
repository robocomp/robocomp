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
from PyQt4.QtGui import QAction, QMenu, QWidget, QLabel

"""
This widget is used to display the details of a component when hovering over the nodes
This contains the GUI and internal process regarding the controlling of the a particular component.
"""
class ItemDetailWidget(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self)
        self.component = None
        self.setParent(parent)
        self.detailString = " "
        self.label = QLabel(self)
        self.label.setGeometry(0, 0, 200, 150)
        self.isShowing = False
        self.setAutoFillBackground(True)
        self.hide()

    def showdetails(self, x, y, item=None):
        self.item = item
        string = ""
        string = string + "Name ::" + item.alias + "\n"
        string = string + "Group Name:: " + item.groupName + "\n"

        self.label.setText(string)
        self.setGeometry(x, y, 150, 150)
        self.isShowing = True
        self.show()

        # def contextMenuEvent(self,event):
        #	print "hello"
        #	GloPos=event.globalPos()
        #	self.hide()
        #	self.parent.graphTree.CompoPopUpMenu.setComponent(self.item.graphicsItem)
        #	self.CompoPopUpMenu.popup(GloPos)


class ComponentMenu(QMenu):
    def __init__(self, parent):
        QMenu.__init__(self, parent)

        self.ActionUp = QAction("Up", parent)
        self.ActionDown = QAction("Down", parent)
        self.ActionEdit = QAction("Edit", parent)
        self.ActionControl = QAction("Control", parent)
        self.ActionNewConnection = QAction("New Connection", parent)
        self.ActionAddToGroup = QAction("Add to Group", parent)
        self.ActionDelete = QAction("Delete", parent)
        self.ActionRemoveFromGroup = QAction("Remove Group", parent)
        self.ActionUpGroup = QAction("UP All", parent)
        self.ActionDownGroup = QAction("DOWN All", parent)
        # self.ActionFreq=QAction("Freq",parent)

        self.GroupMenu = QMenu("Group", parent)
        self.GroupMenu.addAction(self.ActionAddToGroup)
        self.GroupMenu.addAction(self.ActionRemoveFromGroup)
        self.GroupMenu.addAction(self.ActionUpGroup)
        self.GroupMenu.addAction(self.ActionDownGroup)

        # self.addAction(self.ActionFreq)
        self.addAction(self.ActionDelete)
        self.addAction(self.ActionUp)
        self.addAction(self.ActionDown)
        self.addAction(self.ActionNewConnection)
        self.addMenu(self.GroupMenu)
        self.addAction(self.ActionControl)
        self.addAction(self.ActionEdit)
        self.current_component = None

    def set_component(self, component):
        self.current_component = component


##
# This is the menu which appears when right click on the background of the graphics Scene
##

class BackgroundMenu(QMenu):
    def __init__(self, parent):
        QMenu.__init__(self, parent)
        self.ActionSettings = QAction("Settings", parent)
        self.ActionUp = QAction("Up All", parent)
        self.ActionDown = QAction("Down All", parent)
        self.ActionSearch = QAction("Search", parent)
        self.ActionAdd = QAction("Add Component", parent)
        self.ActionNewGroup = QAction("New Group", parent)
        self.ActionStretch = QAction("Stretch", parent)

        self.GraphMenu = QMenu("Graph", parent)
        self.GraphMenu.addAction(self.ActionStretch)

        self.addMenu(self.GraphMenu)
        self.addAction(self.ActionUp)
        self.addAction(self.ActionDown)
        self.addAction(self.ActionAdd)
        self.addAction(self.ActionNewGroup)
        self.addAction(self.ActionSettings)
        self.addAction(self.ActionSearch)
        self.pos = None

    def setPos(self, pos):
        self.pos = pos