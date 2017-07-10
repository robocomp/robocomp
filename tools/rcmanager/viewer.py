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
import math

import logging
import random

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtGui import QGraphicsScene

from widgets import dialogs, code_editor, network_graph, menus

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

class QTextEditLogger(logging.Handler):
    def __init__(self, text_widget):
        super(QTextEditLogger, self).__init__()
        self.text_widget = text_widget

    def emit(self, record):
        self.append_line(self.format(record))  # implementation of append_line omitted

    def append_line(self, msg):
        self.text_widget.append(msg)


MainWindow = uic.loadUiType("formManager.ui")[0]  # Load the UI


class Viewer(QtGui.QMainWindow, MainWindow):
    """docstring for Viewer"""

    def __init__(self, arg=None):
        self._logger = logging.getLogger('RCManager.Viewer')
        self._logger.setLevel(logging.DEBUG)
        print "------------------------------------"
        print "Hello this is Viewer coming up"
        super(Viewer, self).__init__(arg)
        self.setupUi(self)
        h = QTextEditLogger(self.textBrowser)
        format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        h.setFormatter(format)
        self._logger.addHandler(h)

        self.setWindowIcon(QtGui.QIcon(QtGui.QPixmap("share/rcmanager/drawing_green.png")))
        self.showMaximized()
        self.tabWidget.removeTab(0)

        self.componentList = []
        self.networkSettings = network_graph.NetworkGraphicValues()

        self.save_warning = dialogs.SaveWarningDialog(self)

        self.NetworkScene = QGraphicsScene()  # The graphicsScene
        self.graphTree = network_graph.ComponentTree(self.frame, mainclass=self)  # The graphicsNode
        self.NetworkScene.setSceneRect(-15000, -15000, 30000, 30000)
        self.graphTree.setScene(self.NetworkScene)

        self.graphTree.setObjectName(_fromUtf8("graphicsView"))
        self.gridLayout_8.addWidget(self.graphTree, 0, 0, 1, 1)
        self.initialize_zoom()

        # This will read the the network setting from xml files and will set the values
        self.networkSettingDialog = dialogs.NetworkSettingsDialog()
        # tool always works either on opened xml file or user dynamically build xml file.
        # So the two variable given below will always be the negation of each other
        self.FileOpenStatus = False
        self.UserBuiltNetworkStatus = True

        # To track the changes in the network both functionaly and visually
        self.HadChanged = False

        self.LogFileSetter = dialogs.LogFileSetterDialog(self)
        self.simulatorTimer = QtCore.QTimer()

        self.connectionBuilder = dialogs.ConnectionBuilderDialog(self)  # This will take care of connection building between components

        self.position_multiplier_dialog = dialogs.PositionMultiplierDialog()

        self.group_builder_dialog = dialogs.GroupBuilderDialog(self)  # It will help to create a new group
        # setting the code Editor

        self.group_selector_dialog = dialogs.GroupSelectorDialog(self)

        self.CodeEditor = code_editor.CodeEditor.get_code_editor(self.tab_2)
        self.verticalLayout_2.addWidget(self.CodeEditor)

        # The small widget which appears when we right click on a node in tree

        self.node_detail_displayer = menus.ItemDetailWidget(self.graphTree)

        # Setting up the connection
        self.setup_actions()

        self.mid_value_horizontal = 0
        self.mid_value_vertical = 0
        self.currentZoom = 0

    def initialize_zoom(self):  # To connect the slider motion to zooming
        self.verticalSlider.setRange(-20, 20)
        self.verticalSlider.setTickInterval(0.5)
        self.verticalSlider.setValue(0)
        self.currentZoom = 0
        self.verticalSlider.valueChanged.connect(self.graph_zoom)

    def graph_zoom(self):  # To be called when ever we wants to zoomingfactor
        # NoAnchor
        # AnchorViewCenter
        # AnchorUnderMouse

        self.graphTree.setTransformationAnchor(self.graphTree.AnchorUnderMouse)

        new = self.verticalSlider.value()
        diff = new - self.currentZoom
        self.currentZoom = new
        zooming_factor = math.pow(1.2, diff)
        self.graphTree.scale(zooming_factor, zooming_factor)

    def setup_actions(self):  # To setUp connection like saving,opening,etc
        self.connect(self.simulatorTimer, QtCore.SIGNAL("timeout()"), self.simulate)
        # # self.connect(self.toolButton,QtCore.SIGNAL("hovered()"),self.hoverAddComponent)
        # # self.connect(self.toolButton_9,QtCore.SIGNAL("hovered()"),self.hoverXmlSettings)
        # # self.connect(self.toolButton_5,QtCore.SIGNAL("hovered()"),self.hoverPrintDefaultNode)
        # # self.connect(self.toolButton_4,QtCore.SIGNAL("hovered()"),self.hoverPrintDefaultSettings)
        # # self.connect(self.toolButton_3,QtCore.SIGNAL("hovered()"),self.hoverRefreshFromXml)
        # # self.connect(self.toolButton_10,QtCore.SIGNAL("hovered()"),self.hoverNetworkTreeSettings)
        # # self.connect(self.toolButton_6,QtCore.SIGNAL("hovered()"),self.hoverRefreshFromTree)
        # self.connect(self.actionSet_Log_File, QtCore.SIGNAL("triggered(bool)"), self.set_log_file)
        #
        # self.connect(self.tabWidget, QtCore.SIGNAL("currentChanged(int)"), self.tab_index_changed)
        #
        # # File menu buttons
        # self.connect(self.actionSave, QtCore.SIGNAL("triggered(bool)"), self.save_xml_file)
        # self.connect(self.actionOpen, QtCore.SIGNAL("triggered(bool)"), self.open_xml_file)
        # self.connect(self.actionExit, QtCore.SIGNAL("triggered(bool)"), self.exit_rcmanager)
        #
        # # Edit menu buttons
        # self.connect(self.actionSetting, QtCore.SIGNAL("triggered(bool)"), self.rcmanager_setting)
        #
        # # View menu buttons
        # self.connect(self.actionLogger, QtCore.SIGNAL("triggered(bool)"), self.toggle_logger_view)
        # self.connect(self.actionComponent_List, QtCore.SIGNAL("triggered(bool)"), self.toggle_component_list_view)
        # self.connect(self.actionFull_Screen, QtCore.SIGNAL("triggered(bool)"), self.toggle_full_screen_view)
        #
        # self.connect(self.actionON, QtCore.SIGNAL("triggered(bool)"), self.simulator_on)
        # self.connect(self.actionOFF, QtCore.SIGNAL("triggered(bool)"), self.simulator_off)
        # self.connect(self.actionSetting_2, QtCore.SIGNAL("triggered(bool)"), self.simulator_settings)
        # self.connect(self.actionSetting_3, QtCore.SIGNAL("triggered(bool)"), self.control_panel_settings)
        # self.connect(self.actionSetting_4, QtCore.SIGNAL("triggered(bool)"), self.editor_settings)
        # self.connect(self.graphTree.BackPopUpMenu.ActionUp, QtCore.SIGNAL("triggered(bool)"),
        #              self.up_all_components)
        # self.connect(self.graphTree.BackPopUpMenu.ActionDown, QtCore.SIGNAL("triggered(bool)"),
        #              self.down_all_components)
        # self.connect(self.graphTree.BackPopUpMenu.ActionSearch, QtCore.SIGNAL("triggered(bool)"),
        #              self.search_inside_tree)
        # self.connect(self.graphTree.BackPopUpMenu.ActionAdd, QtCore.SIGNAL("triggered(bool)"), self.add_new_node)
        # self.connect(self.graphTree.BackPopUpMenu.ActionSettings, QtCore.SIGNAL("triggered(bool)"),
        #              self.set_network_settings)
        # self.connect(self.graphTree.BackPopUpMenu.ActionNewGroup, QtCore.SIGNAL("triggered(bool)"),
        #              self.add_new_group)
        # self.connect(self.graphTree.BackPopUpMenu.ActionStretch, QtCore.SIGNAL("triggered(bool)"),
        #              self.stretch_graph)
        #
        # self.connect(self.graphTree.CompoPopUpMenu.ActionEdit, QtCore.SIGNAL("triggered(bool)"),
        #              self.edit_selected_component)
        # self.connect(self.graphTree.CompoPopUpMenu.ActionDelete, QtCore.SIGNAL("triggered(bool)"),
        #              self.delete_selected_component)
        # self.connect(self.graphTree.CompoPopUpMenu.ActionUp, QtCore.SIGNAL("triggered(bool)"),
        #              self.up_selected_component)
        # self.connect(self.graphTree.CompoPopUpMenu.ActionAddToGroup, QtCore.SIGNAL("triggered(bool)"),
        #              self.add_component_to_group)
        # self.connect(self.graphTree.CompoPopUpMenu.ActionDown, QtCore.SIGNAL("triggered(bool)"),
        #              self.down_selected_component)
        # self.connect(self.graphTree.CompoPopUpMenu.ActionNewConnection, QtCore.SIGNAL("triggered(bool)"),
        #              self.build_new_connection)
        # self.connect(self.graphTree.CompoPopUpMenu.ActionControl, QtCore.SIGNAL("triggered(bool)"),
        #              self.control_component)
        # self.connect(self.graphTree.CompoPopUpMenu.ActionRemoveFromGroup, QtCore.SIGNAL("triggered(bool)"),
        #              self.component_remove_from_group)
        # self.connect(self.graphTree.CompoPopUpMenu.ActionUpGroup, QtCore.SIGNAL("triggered(bool)"), self.up_group)
        # self.connect(self.graphTree.CompoPopUpMenu.ActionDownGroup, QtCore.SIGNAL("triggered(bool)"),
        #              self.down_group)
        # # self.connect(self.graphTree.CompoPopUpMenu.ActionFreq,QtCore.SIGNAL("triggered(bool)"),self.getFreq)
        #
        # self.connect(self.toolButton_2, QtCore.SIGNAL("clicked()"), self.search_entered_alias)
        # self.connect(self.toolButton_7, QtCore.SIGNAL("clicked()"), self.simulator_on)
        # self.connect(self.toolButton_8, QtCore.SIGNAL("clicked()"), self.simulator_off)
        #
        # self.connect(self.save_warning, QtCore.SIGNAL("save()"), self.save_xml_file)
        # self.connect(self.toolButton_3, QtCore.SIGNAL("clicked()"), self.refresh_tree_from_code)
        # self.connect(self.toolButton_4, QtCore.SIGNAL("clicked()"), self.add_network_templ)
        # self.connect(self.toolButton_5, QtCore.SIGNAL("clicked()"), self.add_component_templ)
        # self.connect(self.toolButton_6, QtCore.SIGNAL("clicked()"), self.refresh_code_from_tree)
        # self.connect(self.toolButton_9, QtCore.SIGNAL("clicked()"), self.editor_font_settings)
        # # self.connect(self.toolButton_10,QtCore.SIGNAL("clicked()"),self.getNetworkSetting)(Once finished Uncomment this)
        # self.connect(self.toolButton, QtCore.SIGNAL("clicked()"), self.add_new_component)

        self._logger.info("Tool started")


    """ Method to animate the nodes to be distributed on the scene """
    def simulate1(self):  # To switch ON simulator::Unfinished
        for iterr in self.componentList:
            force_x = force_y = 0.
            for iterr2 in self.componentList:
                if iterr.alias == iterr2.alias:
                    continue
                ix = iterr.x - iterr2.x
                iy = iterr.y - iterr2.y

                while ix == 0 and iy == 0:
                    iterr.x = iterr.x + random.uniform(-10, 10)
                    iterr2.x = iterr2.x + random.uniform(-10, 10)
                    iterr.y = iterr.y + random.uniform(-10, 10)
                    iterr2.y = iterr2.y + random.uniform(-10, 10)
                    ix = iterr.x - iterr2.x
                    iy = iterr.y - iterr2.y

                angle = math.atan2(iy, ix)
                dist2 = ((abs((iy * iy) + (ix * ix))) ** 0.5) ** 2.
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
                    force = math.sqrt(abs((iy * iy) + (ix * ix)))  # force means distance actually
                    # if force <= self.spring_length: continue       # "
                    force -= self.networkSettings.spring_length  # force means spring strain now
                    force = force * self.networkSettings.hookes_constant  # now force means force :-)
                    force_x -= force * math.cos(angle)
                    force_y -= force * math.sin(angle)

            iterr.vel_x = (iterr.vel_x + (force_x * self.networkSettings.time_elapsed2)) * self.networkSettings.roza
            iterr.vel_y = (iterr.vel_y + (force_y * self.networkSettings.time_elapsed2)) * self.networkSettings.roza

        # Update positions
        for iterr in self.componentList:
            iterr.x += iterr.vel_x
            iterr.y += iterr.vel_y

        for iterr in self.componentList:
            # print "updating "+iterr.alias
            iterr.graphicsItem.setPos(QtCore.QPointF(iterr.x, iterr.y))
            iterr.graphicsItem.updateforDrag()

    """ Method to animate the nodes to be distributed on the scene """
    def simulate(self):
        optimal_gap = 500
        tolerance = 200

        repulsion_factor = (optimal_gap - tolerance) ** 2.1
        attraction_factor = 0
        damping_factor = 1

        for i in self.componentList:

            attractive_force_x = 0
            attractive_force_y = 0
            repulsive_force_x = 0
            repulsive_force_y = 0
            # net_force_x = 0
            # net_force_y = 0
            # dx = 0
            # dy = 0

            for j in self.componentList:
                if i.alias == j.alias:
                    continue

                distance = ((i.x - j.x) ** 2 + (i.y - j.y) ** 2) ** 0.5

                if distance == 0:
                    distance = 0.1

                attractive_force_x += attraction_factor * (j.x - i.x)
                attractive_force_y += attraction_factor * (j.y - i.y)

                repulsive_force_x += repulsion_factor * (i.x - j.x) / (distance ** 3)
                repulsive_force_y += repulsion_factor * (i.y - j.y) / (distance ** 3)

            net_force_x = attractive_force_x + repulsive_force_x
            net_force_y = repulsive_force_x + repulsive_force_y

            dx = net_force_x * damping_factor
            dy = net_force_y * damping_factor

            dx = int(dx)
            dy = int(dy)

            i.vel_x = i.x + dx
            i.vel_y = i.y + dy

        for i in self.componentList:
            i.x = i.vel_x
            i.y = i.vel_y

        for i in self.componentList:
            i.graphicsItem.setPos(QtCore.QPointF(i.x, i.y))
            i.graphicsItem.updateforDrag()


