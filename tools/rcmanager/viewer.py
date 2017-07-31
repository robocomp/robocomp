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
import random

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtGui import QGraphicsScene, QPushButton, QBrush, QColor
from widgets import dialogs, code_editor, network_graph, menus
from widgets.QNetworkxGraph.QNetworkxGraph import QNetworkxWidget
from logger import RCManagerLogger

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

MainWindow = uic.loadUiType("formManager.ui")[0]  # Load the UI

class Viewer(QtGui.QMainWindow, MainWindow):
    """docstring for Viewer"""
    def __init__(self, rcmanagerSignals=None):
        self._logger = RCManagerLogger().get_logger("RCManager.Viewer")
        self.rcmanagerSignals = rcmanagerSignals

        super(Viewer, self).__init__()
        self.setupUi(self)
        # RCManagerLogger().set_text_edit_handler(self.textBrowser)
        self.setWindowIcon(QtGui.QIcon(QtGui.QPixmap("share/rcmanager/drawing_green.png")))
        self.showMaximized()
        self.tabWidget.removeTab(0)

        self.componentList = []
        self.networkSettings = network_graph.NetworkGraphicValues()

        self.save_warning = dialogs.SaveWarningDialog(self)

        self.add_graph_visualization()
        self.initialize_zoom()

        # This will read the the network setting from xml files and will set the values
        self.networkSettingDialog = dialogs.NetworkSettingsDialog()
        # tool always works either on opened xml file or user dynamically build xml file.
        # So the two variable given below will always be the negation of each other
        self.FileOpenStatus = False
        self.UserBuiltNetworkStatus = True

        # To track the changes in the network both functionaly and visually
        self.HadChanged = False

        self.log_file_setter = dialogs.LogFileSetterDialog(self)
        self.simulatorTimer = QtCore.QTimer()

        self.connectionBuilder = dialogs.ConnectionBuilderDialog(self)  # This will take care of connection building between components

        self.position_multiplier_dialog = dialogs.PositionMultiplierDialog()

        self.group_builder_dialog = dialogs.GroupBuilderDialog(self)  # It will help to create a new group
        # setting the code Editor

        self.group_selector_dialog = dialogs.GroupSelectorDialog(self)

        self.CodeEditor = code_editor.CodeEditor.get_code_editor(self.tab_2)
        self.verticalLayout_2.addWidget(self.CodeEditor)

        # The small widget which appears when we right click on a node in tree

        self.node_detail_displayer = menus.ItemDetailWidget(self.graph_visualization)

        # Setting up the connection
        self.setup_actions()

        self.mid_value_horizontal = 0
        self.mid_value_vertical = 0
        self.currentZoom = 0

        self.rcmanagerSignals.viewerIsReady.emit()

        # self.actionOpen.triggered.connect(self.fuuu)

    def initialize_zoom(self):  # To connect the slider motion to zooming
        self.verticalSlider.setRange(-20, 20)
        self.verticalSlider.setTickInterval(0.5)
        self.verticalSlider.setValue(0)
        self.currentZoom = 0
        self.verticalSlider.valueChanged.connect(self.graph_zoom)

    def graph_zoom(self):  # To be called whenever we wants to zoom
        # NoAnchor
        # AnchorViewCenter
        # AnchorUnderMouse

        self.graph_visualization.setTransformationAnchor(self.graph_visualization.AnchorViewCenter)

        new = self.verticalSlider.value()
        diff = new - self.currentZoom
        self.currentZoom = new
        zooming_factor = math.pow(1.2, diff)
        self.graph_visualization.scale_view(scale_factor=zooming_factor)

    def setup_actions(self):  # To setUp connection like saving,opening,etc
        # setup rcmanager signals
        self.rcmanagerSignals.addNode.connect(self.add_node)

        # self.connect(self.simulatorTimer, QtCore.SIGNAL("timeout()"), self.simulate)
        # # self.connect(self.toolButton,QtCore.SIGNAL("hovered()"),self.hoverAddComponent)
        # # self.connect(self.toolButton_9,QtCore.SIGNAL("hovered()"),self.hoverXmlSettings)
        # # self.connect(self.toolButton_5,QtCore.SIGNAL("hovered()"),self.hoverPrintDefaultNode)
        # # self.connect(self.toolButton_4,QtCore.SIGNAL("hovered()"),self.hoverPrintDefaultSettings)
        # # self.connect(self.toolButton_3,QtCore.SIGNAL("hovered()"),self.hoverRefreshFromXml)
        # # self.connect(self.toolButton_10,QtCore.SIGNAL("hovered()"),self.hoverNetworkTreeSettings)
        # # self.connect(self.toolButton_6,QtCore.SIGNAL("hovered()"),self.hoverRefreshFromTree)
        self.connect(self.actionSet_Log_File, QtCore.SIGNAL("triggered(bool)"), self.set_log_file)
        #
        self.connect(self.tabWidget, QtCore.SIGNAL("currentChanged(int)"), self.tab_index_changed)
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
        self.connect(self.actionLogger, QtCore.SIGNAL("triggered(bool)"), self.toggle_logger_view)
        self.connect(self.actionComponent_List, QtCore.SIGNAL("triggered(bool)"), self.toggle_component_list_view)
        self.connect(self.actionFull_Screen, QtCore.SIGNAL("triggered(bool)"), self.toggle_full_screen_view)
        self.actionFull_Screen.setShortcut("F11")

        self.connect(self.actionSet_Color, QtCore.SIGNAL("triggered(bool)"), self.color_picker)
        #
        self.connect(self.actionON, QtCore.SIGNAL("triggered(bool)"), self.graph_visualization.start_animation)
        self.connect(self.actionOFF, QtCore.SIGNAL("triggered(bool)"), self.graph_visualization.stop_animation)
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

        # self._logger.info("Tool started")

    # Background color picker widget
    def color_picker(self):
        color = QtGui.QColorDialog.getColor()
        self.graph_visualization.background_color = color
        self.graph_visualization.setBackgroundBrush(color)

    # View menu functions begin
    def toggle_logger_view(self):
        if self.actionLogger.isChecked():
            self.dockWidget.show()
            self.actionFull_Screen.setChecked(False)
        else:
            self.dockWidget.hide()
            self.actionFull_Screen.setChecked(not self.actionComponent_List.isChecked())

    def toggle_component_list_view(self):
        if self.actionComponent_List.isChecked():
            self.dockWidget_2.show()
            self.actionFull_Screen.setChecked(False)
        else:
            self.dockWidget_2.hide()
            self.actionFull_Screen.setChecked(not self.actionLogger.isChecked())

    def toggle_full_screen_view(self):
        if self.actionFull_Screen.isChecked():
            self.actionLogger.setChecked(False)
            self.actionComponent_List.setChecked(False)

            self.toggle_logger_view()
            self.toggle_component_list_view()
        else:
            self.actionLogger.setChecked(True)
            self.actionComponent_List.setChecked(True)

            self.toggle_logger_view()
            self.toggle_component_list_view()

    # Generate start / stop signals for components
    def send_start_signal(self):
        selectedNodes = self.graph_visualization.selected_nodes()

        for i in selectedNodes:
            self.rcmanagerSignals.startComponent.emit(i)

    def send_stop_signal(self):
        selectedNodes = self.graph_visualization.selected_nodes()

        for i in selectedNodes:
            self.rcmanagerSignals.stopComponent.emit(i)

    def add_node(self, node, nodedata=None):
        self._logger.info("The viewer received signal to draw component: " + node)
        createdNode = self.graph_visualization.add_node(node)

        # Start / stop context menu options
        menu = dict()
        menu['Start'] = (self, "send_start_signal")
        menu['Stop'] = (self, "send_stop_signal")
        createdNode.add_context_menu(menu)

    def add_edge(self, orig_node, dest_node, edge_data=None):
        self._logger.info("The viewer received signal to draw edge from: " + orig_node + " to: " + dest_node)
        self.graph_visualization.add_edge(first_node=orig_node, second_node=dest_node)

    def set_log_file(self):
        self.log_file_setter.setFile()

    def tab_index_changed(self):  # This will make sure the common behavior is not working unneccessarily
        index = self.tabWidget.currentIndex()
        if index == 1 or index == 2:  # CommonProxy should only work if the first tab is visible
            if self.currentComponent is not None:
                self.currentComponent.CommonProxy.set_visibility(False)

    def add_graph_visualization(self):
        # self.NetworkScene = QGraphicsScene()  # The graphicsScene
        # self.graphTree = network_graph.ComponentTree(self.frame, mainclass=self)  # The graphicsNode
        # self.NetworkScene.setSceneRect(-15000, -15000, 30000, 30000)
        # self.graphTree.setScene(self.NetworkScene)
        #
        # self.graphTree.setObjectName(_fromUtf8("graphicsView"))

        self.graph_visualization = QNetworkxWidget()
        self.gridLayout_8.addWidget(self.graph_visualization, 0, 0, 1, 1)

    def get_graph_nodes_positions(self):
        return self.graph_visualization.get_current_nodes_positions()
