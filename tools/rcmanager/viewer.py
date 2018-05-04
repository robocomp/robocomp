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
import xmlreader

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtGui import QGraphicsScene, QPushButton, QBrush, QColor
from widgets import dialogs, code_editor, network_graph, menus
from widgets.QNetworkxGraph.QNetworkxGraph import QNetworkxWidget, NodeShapes
from logger import RCManagerLogger
from rcmanagerSignals import CustomSignalCollection

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

MainWindow = uic.loadUiType("formManager.ui")[0]  # Load the UI

class Viewer(QtGui.QMainWindow, MainWindow):
    """This is the Viewer object for our MVC model. It is responsible for
    creating the UI and displaying the graph and the corresponding xml
    file. It uses a signal slot mechanism to communicate to the controller."""

    def __init__(self):
        super(Viewer, self).__init__()

        # logger object for viewer
        self._logger = RCManagerLogger().get_logger("RCManager.Viewer")

        # setup rcmanager UI
        self.setupUi(self)

        # set the text window to display the log data
        RCManagerLogger().set_text_edit_handler(self.textBrowser)

        # set rcmanager window icon
        self.setWindowIcon(QtGui.QIcon(QtGui.QPixmap("share/rcmanager/drawing_green.png")))
        self.showMaximized()

        # removing the tab named 'Control panel' in formManager.ui
        # this tab has no implementation as of now
        self.tabWidget.removeTab(0)

        # set the code editor object to display the xml file in the tab named 'Text'
        self.codeEditor = code_editor.CodeEditor.get_code_editor(self.tab_2)
        self.verticalLayout_2.addWidget(self.codeEditor)

        # initialise graph object
        self.add_graph_visualization()

        # connect the various UI components (buttons etc.) to the relevant functions (slots)
        self.setup_actions()

        # temporary code
        # self.actionComponent_List.toggle()
        # self.toggle_component_list_view()

        CustomSignalCollection.viewerIsReady.emit()

    def setup_actions(self):
        # tab widget signals 
        self.connect(self.tabWidget, QtCore.SIGNAL("currentChanged(int)"), self.tab_index_changed)

        # file menu buttons
        self.connect(self.actionSave, QtCore.SIGNAL("triggered(bool)"), lambda: self.save_model(False))
        self.connect(self.actionSave_As, QtCore.SIGNAL("triggered(bool)"), lambda: self.save_model(True))
        self.connect(self.actionOpen, QtCore.SIGNAL("triggered(bool)"), self.open_model)
        self.connect(self.actionExit, QtCore.SIGNAL("triggered(bool)"), self.close_model)

        # edit menu buttons

        # view menu buttons
        self.connect(self.actionLogger, QtCore.SIGNAL("triggered(bool)"), self.toggle_logger_view)
        self.connect(self.actionComponent_List, QtCore.SIGNAL("triggered(bool)"), self.toggle_component_list_view)
        self.connect(self.actionFull_Screen, QtCore.SIGNAL("triggered(bool)"), self.toggle_full_screen_view)

        # tools menu buttons
        self.connect(self.actionSet_Color, QtCore.SIGNAL("triggered(bool)"), self.set_background_color)
        self.connect(self.actionON, QtCore.SIGNAL("triggered(bool)"), self.graph_visualization.start_animation)
        self.connect(self.actionOFF, QtCore.SIGNAL("triggered(bool)"), self.graph_visualization.stop_animation)

    # background color picker widget
    def set_background_color(self, color=None):
        if not color:
            color = QtGui.QColorDialog.getColor()
        self.graph_visualization.background_color = color
        self.graph_visualization.setBackgroundBrush(color)

    # view menu functions
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

    # file menu functions
    def save_model(self, saveAs=True):
        # if we are currently on the editor tab, remember to update the graph before saving
        # the save file is always generated from the actual graph rather than the editor text
        # this is to avoid any possible syntax errors that might be present in the editor text 
        index = self.tabWidget.currentIndex()
        if index == 1:
            self.refresh_graph_from_editor()

        if saveAs:
            filename = QtGui.QFileDialog.getSaveFileName(self, 'Save File', self.filename)
        else:
            filename = self.filename
        CustomSignalCollection.saveModel.emit(filename)
        self.dirtyBit = False
        
    def open_model(self):
        filename = QtGui.QFileDialog.getOpenFileName(self, 'Open File')
        CustomSignalCollection.openModel.emit(filename, True)

    def close_model(self):
        self.close()

    def closeEvent(self, QCloseEvent):
        CustomSignalCollection.closeModel.emit()
        self.save_before_quit_prompt(QCloseEvent)

    # component list functions

    # add buttons to the component list
    def add_to_component_list(self, node): 
        button = QPushButton(node, parent=None)
        button.clicked.connect(lambda: self.on_button_click(node))
        self.verticalLayout.insertWidget(self.verticalLayout.count()-1, button)

    def on_button_click(self, node):
        # select the node corresponding to the clicked button
        self.graph_visualization.clear_selection()
        nodeGraphicItem = self.graph_visualization.get_node(node)['item']
        nodeGraphicItem.setSelected(True)

        # center the scene over the selected node
        nodePosition = [nodeGraphicItem.pos().x(), nodeGraphicItem.pos().y()]
        self.graph_visualization.center_on(nodePosition)

    # this does not look like production-level code
    # please try to use in-built functions of Qt and make this segment a bit more cleaner
    def remove_from_component_list(self, node): 
        numItems = self.verticalLayout.count()
        for index in range(numItems):
            button = self.verticalLayout.itemAt(index).widget()
            if button.text()==str(node): 
               button.setParent(None)
               button.deleteLater()
               break

    # clears the entire component list
    def clear_component_list(self):
        while self.verticalLayout.count()>1:
            button = self.verticalLayout.itemAt(0).widget()
            button.setParent(None)
            button.deleteLater()

    # node/edge add/remove
    def remove_node(self, node):
        self.graph_visualization.remove_node(str(node))
        self.remove_from_component_list(str(node))

    def add_node(self, node, nodedata=None, position=None):
        self._logger.info("The viewer received signal to draw component: " + node)
        self.graph_visualization.add_node(node, position)
        createdNode = self.graph_visualization.get_node(node)['item']

        # start / stop context menu options
        menu = dict()
        menu['Start'] = (CustomSignalCollection.startComponent, 'emit')
        menu['Stop'] = (CustomSignalCollection.stopComponent, 'emit')
        menu['Remove component'] = (CustomSignalCollection.removeComponent, 'emit')
        createdNode.add_context_menu(menu)

        if 'componentType' in nodedata.keys():
            if str(nodedata['componentType']['@value']) == 'agent':
                createdNode.set_node_shape(NodeShapes.SQUARE)
                return

        createdNode.set_node_shape(NodeShapes.CIRCLE)
        self.add_to_component_list(node)

    def add_edge(self, orig_node, dest_node, edge_data=None):
        self._logger.info("The viewer received signal to draw edge from: " + orig_node + " to: " + dest_node)
        self.graph_visualization.add_edge(first_node=orig_node, second_node=dest_node)

    def update_node_profile(self, componentAlias, node_profile):
        node = self.graph_visualization.get_node(componentAlias)['item']
        node.set_node_profile(node_profile)

    def tab_index_changed(self):
        index = self.tabWidget.currentIndex()

        if index == 0:
            self.refresh_graph_from_editor()
        elif index == 1:
            self.refresh_editor_from_graph()

    def refresh_graph_from_editor(self):
        xml = str(self.codeEditor.text())

        if not xmlreader.validate_xml(xml):
            return

        self.clear_component_list()

        filename = '.temp.xml'
        open(filename, 'w').close()
        fileDescriptor = open(filename, 'a')
        fileDescriptor.write(xml)
        fileDescriptor.close()

        CustomSignalCollection.openModel.emit(filename, False)

    def refresh_editor_from_graph(self):
        filename = '.temp.xml'
        CustomSignalCollection.saveModel.emit(filename)
        file = open(filename, 'r')
        xml = file.read()
        self.set_editor_text(xml)

    def add_graph_visualization(self):
        self.graph_visualization = QNetworkxWidget()

        # context menu options
        menu = dict()
        menu['Change Background Color'] = (self, "set_background_color")
        menu['Remove components'] = (CustomSignalCollection.removeComponent, "emit")
        self.graph_visualization.add_context_menu(menu)

        self.gridLayout_8.addWidget(self.graph_visualization, 0, 0, 1, 1)

    def clear_graph_visualization(self):
        self.graph_visualization.clear()

    def set_editor_text(self, text):
        if text is not None:
            self.codeEditor.setText(text)
        else:
            self._logger.error("Text object received is NoneType")

    def get_graph_nodes_positions(self):
        return self.graph_visualization.get_current_nodes_positions()

    # user prompts 
    def save_before_quit_prompt(self, QCloseEvent):
        if self.dirtyBit:
            quit_msg = "There are unsaved changes. Do you want to save before exiting?"
            reply = QtGui.QMessageBox.question(self, 'Save Model?',
                                               quit_msg, QtGui.QMessageBox.No, QtGui.QMessageBox.Cancel, QtGui.QMessageBox.Yes)

            if reply == QtGui.QMessageBox.Yes:
                self.save_model()
                QCloseEvent.accept()
            elif reply == QtGui.QMessageBox.No:
                QCloseEvent.accept()
            elif reply == QtGui.QMessageBox.Cancel:
                QCloseEvent.ignore()
        else:
            QCloseEvent.accept()
