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
import os
import random
import sys
import time
import signal

from PyQt4 import QtCore, QtGui, Qt, uic

import rcmanagerConfig

signal.signal(signal.SIGINT, signal.SIG_DFL)

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
        self.ipList = []  # Ip listed from the xml file
        self.currentComponent = None
        self.setWindowIcon(QtGui.QIcon(QtGui.QPixmap("resources/icons/drawing_green.png")))
        self.showMaximized()
        self.componentList = []
        self.networkSettings = rcmanagerConfig.NetworkValues()

        self.setupUi(self)
        self.tabWidget.removeTab(0)
        self.Logger = rcmanagerConfig.Logger(self.textBrowser)

        self.SaveWarning = rcmanagerConfig.SaveWarningDialog(self)

        self.NetworkScene = rcmanagerConfig.ComponentScene(self)  # The graphicsScene
        self.graphTree = rcmanagerConfig.ComponentTree(self.frame, mainclass=self)  # The graphicsNode
        self.NetworkScene.setSceneRect(-15000, -15000, 30000, 30000)
        self.graphTree.setScene(self.NetworkScene)

        self.graphTree.setObjectName(_fromUtf8("graphicsView"))
        self.gridLayout_8.addWidget(self.graphTree, 0, 0, 1, 1)
        self.set_zoom()

        # This will read the the network setting from xml files and will set the values
        self.networkSettingDialog = rcmanagerConfig.NetworkSettings(self)
        # tool always works either on opened xml file or user dynamically build xml file.
        # So the two variable given below will always be the negation of each other
        self.FileOpenStatus = False
        self.UserBuiltNetworkStatus = True

        # To track the changes in the network both functionaly and visually
        self.HadChanged = False

        self.LogFileSetter = rcmanagerConfig.LogFileSetter(self, self.Logger)
        self.simulatorTimer = QtCore.QTimer()

        self.connectionBuilder = rcmanagerConfig.connectionBuilder(self,
                                                                   self.Logger)  # This will take care of connection building between components

        self.PositionMultiplier = rcmanagerConfig.PositionMultiplier(self.Logger)

        self.groupBuilder = rcmanagerConfig.GroupBuilder(self, self.Logger)  # It will help to create a new group
        # setting the code Editor

        self.groupSelector = rcmanagerConfig.GroupSelector(self, self.Logger)

        self.CodeEditor = rcmanagerConfig.CodeEditor.get_code_editor(self.tab_2)
        self.verticalLayout_2.addWidget(self.CodeEditor)

        # The small widget which appears when we right click on a node in tree

        self.node_detail_displayer = rcmanagerConfig.ShowItemDetails(self.graphTree)

        # Setting up the connection
        self.setup_actions()

        self.mid_value_horizontal = 0
        self.mid_value_vertical = 0

        # self.textEdit=QtGui.QTextEdit()#Temp
    # self.tabWidget_2.addTab(self.textEdit,"Helllo")#Temp
    # print "Count is "+ str(self.verticalLayout.count())
    # self.toolButton_6.setMouseTracking(True)

    def setup_actions(self):  # To setUp connection like saving,opening,etc
        self.connect(self.simulatorTimer, QtCore.SIGNAL("timeout()"), self.simulate)
        # self.connect(self.toolButton,QtCore.SIGNAL("hovered()"),self.hoverAddComponent)
        # self.connect(self.toolButton_9,QtCore.SIGNAL("hovered()"),self.hoverXmlSettings)
        # self.connect(self.toolButton_5,QtCore.SIGNAL("hovered()"),self.hoverPrintDefaultNode)
        # self.connect(self.toolButton_4,QtCore.SIGNAL("hovered()"),self.hoverPrintDefaultSettings)
        # self.connect(self.toolButton_3,QtCore.SIGNAL("hovered()"),self.hoverRefreshFromXml)
        # self.connect(self.toolButton_10,QtCore.SIGNAL("hovered()"),self.hoverNetworkTreeSettings)
        # self.connect(self.toolButton_6,QtCore.SIGNAL("hovered()"),self.hoverRefreshFromTree)
        self.connect(self.actionSet_Log_File, QtCore.SIGNAL("triggered(bool)"), self.set_log_file)

        self.connect(self.tabWidget, QtCore.SIGNAL("currentChanged(int)"), self.tab_index_changed)

        # File menu buttons
        self.connect(self.actionSave, QtCore.SIGNAL("triggered(bool)"), self.save_xml_file)
        self.connect(self.actionOpen, QtCore.SIGNAL("triggered(bool)"), self.open_xml_file)
        self.connect(self.actionExit, QtCore.SIGNAL("triggered(bool)"), self.exit_rcmanager)

        # Edit menu buttons
        self.connect(self.actionSetting, QtCore.SIGNAL("triggered(bool)"), self.rcmanager_setting)

        # View menu buttons
        self.connect(self.actionLogger, QtCore.SIGNAL("triggered(bool)"), self.toggle_logger_view)
        self.connect(self.actionComponent_List, QtCore.SIGNAL("triggered(bool)"), self.toggle_component_list_view)
        self.connect(self.actionFull_Screen, QtCore.SIGNAL("triggered(bool)"), self.toggle_full_screen_view)

        self.connect(self.actionON, QtCore.SIGNAL("triggered(bool)"), self.simulator_on)
        self.connect(self.actionOFF, QtCore.SIGNAL("triggered(bool)"), self.simulator_off)
        self.connect(self.actionSetting_2, QtCore.SIGNAL("triggered(bool)"), self.simulator_settings)
        self.connect(self.actionSetting_3, QtCore.SIGNAL("triggered(bool)"), self.control_panel_settings)
        self.connect(self.actionSetting_4, QtCore.SIGNAL("triggered(bool)"), self.editor_settings)
        self.connect(self.graphTree.BackPopUpMenu.ActionUp, QtCore.SIGNAL("triggered(bool)"), self.up_all_components)
        self.connect(self.graphTree.BackPopUpMenu.ActionDown, QtCore.SIGNAL("triggered(bool)"),
                     self.down_all_components)
        self.connect(self.graphTree.BackPopUpMenu.ActionSearch, QtCore.SIGNAL("triggered(bool)"), self.search_inside_tree)
        self.connect(self.graphTree.BackPopUpMenu.ActionAdd, QtCore.SIGNAL("triggered(bool)"), self.add_new_node)
        self.connect(self.graphTree.BackPopUpMenu.ActionSettings, QtCore.SIGNAL("triggered(bool)"),
                     self.set_network_settings)
        self.connect(self.graphTree.BackPopUpMenu.ActionNewGroup, QtCore.SIGNAL("triggered(bool)"), self.add_new_group)
        self.connect(self.graphTree.BackPopUpMenu.ActionStretch, QtCore.SIGNAL("triggered(bool)"), self.stretch_graph)

        self.connect(self.graphTree.CompoPopUpMenu.ActionEdit, QtCore.SIGNAL("triggered(bool)"),
                     self.edit_selected_component)
        self.connect(self.graphTree.CompoPopUpMenu.ActionDelete, QtCore.SIGNAL("triggered(bool)"),
                     self.delete_selected_component)
        self.connect(self.graphTree.CompoPopUpMenu.ActionUp, QtCore.SIGNAL("triggered(bool)"),
                     self.up_selected_component)
        self.connect(self.graphTree.CompoPopUpMenu.ActionAddToGroup, QtCore.SIGNAL("triggered(bool)"),
                     self.add_component_to_group)
        self.connect(self.graphTree.CompoPopUpMenu.ActionDown, QtCore.SIGNAL("triggered(bool)"),
                     self.down_selected_component)
        self.connect(self.graphTree.CompoPopUpMenu.ActionNewConnection, QtCore.SIGNAL("triggered(bool)"),
                     self.build_new_connection)
        self.connect(self.graphTree.CompoPopUpMenu.ActionControl, QtCore.SIGNAL("triggered(bool)"),
                     self.control_component)
        self.connect(self.graphTree.CompoPopUpMenu.ActionRemoveFromGroup, QtCore.SIGNAL("triggered(bool)"),
                     self.component_remove_from_group)
        self.connect(self.graphTree.CompoPopUpMenu.ActionUpGroup, QtCore.SIGNAL("triggered(bool)"), self.up_group)
        self.connect(self.graphTree.CompoPopUpMenu.ActionDownGroup, QtCore.SIGNAL("triggered(bool)"), self.down_group)
        # self.connect(self.graphTree.CompoPopUpMenu.ActionFreq,QtCore.SIGNAL("triggered(bool)"),self.getFreq)

        self.connect(self.toolButton_2, QtCore.SIGNAL("clicked()"), self.search_entered_alias)
        self.connect(self.toolButton_7, QtCore.SIGNAL("clicked()"), self.simulator_on)
        self.connect(self.toolButton_8, QtCore.SIGNAL("clicked()"), self.simulator_off)

        self.connect(self.SaveWarning, QtCore.SIGNAL("save()"), self.save_xml_file)
        self.connect(self.toolButton_3, QtCore.SIGNAL("clicked()"), self.refresh_tree_from_code)
        self.connect(self.toolButton_4, QtCore.SIGNAL("clicked()"), self.add_network_templ)
        self.connect(self.toolButton_5, QtCore.SIGNAL("clicked()"), self.add_component_templ)
        self.connect(self.toolButton_6, QtCore.SIGNAL("clicked()"), self.refresh_code_from_tree)
        self.connect(self.toolButton_9, QtCore.SIGNAL("clicked()"), self.editor_font_settings)
        # self.connect(self.toolButton_10,QtCore.SIGNAL("clicked()"),self.getNetworkSetting)(Once finished Uncomment this)
        self.connect(self.toolButton, QtCore.SIGNAL("clicked()"), self.add_new_component)

        self.Logger.logData("Tool started")

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

    # View menu functions end

    def get_freq(self):
        comp = self.graphTree.CompoPopUpMenu.currentComponent.parent
        comp.CheckItem.get_freq()

    def edit_selected_component(self):
        self.tabWidget.setCurrentIndex(1)
        self.CodeEditor.findFirst(self.graphTree.CompoPopUpMenu.currentComponent.parent.alias, False, True, True, True)

    def set_log_file(self):
        self.LogFileSetter.setFile()

    def stretch_graph(self):
        self.PositionMultiplier.updateStretch(self.componentList, self.networkSettings)

    def up_group(self):
        component = self.graphTree.CompoPopUpMenu.currentComponent.parent
        if component.group is not None:
            component.group.up_group_components(self.Logger)
        else:
            self.Logger.logData("No group", "R")

    def down_group(self):
        component = self.graphTree.CompoPopUpMenu.currentComponent.parent
        if component.group is not None:
            component.group.down_group_components(self.Logger)
        else:
            self.Logger.logData("No group", "R")

    def component_remove_from_group(self):
        component = self.graphTree.CompoPopUpMenu.currentComponent.parent
        if component.group is not None:
            component.group.remove_component(component)
            self.NetworkScene.update()
            self.refresh_code_from_tree()
        else:
            self.Logger.logData("No group", "R")

    def add_component_to_group(self):
        component = self.graphTree.CompoPopUpMenu.currentComponent.parent
        self.groupSelector.openSelector(component, self.networkSettings.Groups)
        self.NetworkScene.update()
        self.refresh_code_from_tree()

    def build_new_connection(self):
        self.graphTree.connection_building_status = True
        print "Connection Building"
        self.connectionBuilder.buildNewConnection()
        self.connectionBuilder.setBeg(self.graphTree.CompoPopUpMenu.currentComponent.parent)
        self.connectionBuilder.show()

    def add_new_group(self):
        self.groupBuilder.startBuildGroup(self.networkSettings)

    def delete_selected_component(self):
        self.delete_component(self.graphTree.CompoPopUpMenu.currentComponent.parent)

    def tab_index_changed(self):  # This will make sure the common behavior is not working unneccessarily
        index = self.tabWidget.currentIndex()
        if index == 1 or index == 2:  # CommonProxy should only work if the first tab is visible
            if self.currentComponent is not None:
                self.currentComponent.CommonProxy.set_visibility(False)

    def add_network_templ(self):
        string = rcmanagerConfig.get_default_settings()
        pos = self.CodeEditor.getCursorPosition()
        self.CodeEditor.insertAt(string, pos[0], pos[1])

    def get_network_setting(self):  # This will show the Network setting Dialog box and will help to update the stuffs
        self.networkSettingDialog.setData(self.networkSettings)
        self.networkSettingDialog.show()

    def editor_font_settings(self):  # BUUUUUUUUUUUUUGGGG
        font, ok = QtGui.QFontDialog.getFont()
        if ok:
            self.CodeEditor.setFont(font)
            self.CodeEditor.font = font
            self.Logger.logData("New font set for code editor")

    def refresh_code_from_tree(self):
        self.HadChanged = True
        string = rcmanagerConfig.get_xml_from_network(self.networkSettings, self.componentList, self.Logger)
        self.CodeEditor.setText(string)
        self.Logger.logData("Code updated successfully from the graph")
        self.center_align_graph()

    def refresh_tree_from_code(self, first_time=False):  # This will refresh the code (Not to file)and draw the new tree
        # print "Refreshing"
        try:
            comp_list, settings = rcmanagerConfig.get_data_from_string(str(self.CodeEditor.text()), self.Logger)
        except Exception, e:
            self.Logger.logData("Error while updating tree from code::" + str(e), "R")
        else:

            if first_time is True:
                self.remove_all_components()
                self.NetworkScene.clear()
                self.networkSettings = settings
                self.componentList = comp_list
                try:
                    # if self.areTheyTooClose()==True:
                    # self.theyAreTooClose()
                    self.currentComponent = self.componentList[0]
                    self.ip_count()
                    self.set_all_ip_color()
                    self.set_all_graphics_data()
                    self.draw_all_components()
                    self.set_connection_items()
                    self.draw_all_connection()
                    self.set_component_variables()
                    self.set_directory_items()
                    self.FileOpenStatus = True
                    self.UserBuiltNetworkStatus = True
                    # self.HadChanged=False
                    self.Logger.logData("File updated successfully from the code editor")
                    self.refresh_code_from_tree()

                except Exception, e:
                    self.Logger.logData("File update from code failed " + str(e), "R")

            else:
                self.networkSettings = settings
                for x in comp_list:
                    try:
                        comp = self.search_for_component(x.alias)
                    except:
                        self.componentList.append(x)  # This will add a new component

                for x in self.componentList:
                    if not self.search_inside_list(comp_list, x.alias):
                        self.Logger.logData("Deleted the older component ::" + x.alias)  # If new tree does have this
                        self.delete_component(x)

                for x in self.componentList:
                    for y in comp_list:
                        if x.alias == y.alias:
                            self.copy_and_update(x, y)
                self.Logger.logData("Tree updated successfully from file")

        self.center_align_graph()

    def center_align_graph(self):
        self.mid_value_horizontal = (
                                      self.graphTree.horizontalScrollBar().maximum() + self.graphTree.horizontalScrollBar().minimum()) / 2
        self.mid_value_vertical = (
                                    self.graphTree.verticalScrollBar().maximum() + self.graphTree.verticalScrollBar().minimum()) / 2

        self.graphTree.horizontalScrollBar().setValue(self.mid_value_horizontal)
        self.graphTree.verticalScrollBar().setValue(self.mid_value_vertical)

        self.verticalSlider.setValue(0)
        self.graph_zoom()

    def copy_and_update(self, original, temp):
        if original.x != temp.x or original.y != temp.y:
            original.x = temp.x
            original.y = temp.y
            original.graphicsItem.setPos(original.x, original.y)
            original.graphicsItem.updateforDrag()
            self.Logger.logData("Position updated of ::" + original.alias)

        original.workingdir = temp.workingdir
        original.compup = temp.compup
        original.compdown = temp.compdown
        original.configFile = temp.configFile
        original.nodeColor = temp.nodeColor

        if original.groupName != temp.groupName:
            try:
                group = rcmanagerConfig.search_for_group_name(self.networkSettings, original.groupName)
                group.add_component(original)
            except Exception, e:
                self.Logger.logData(str(e), "R")

        for x in temp.dependences:  # For adding new connection if needed
            if original.dependences.__contains__(x) is False:
                original.dependences.append(x)
                comp = searchforComponent(x)
                self.set_connection(comp, original)
        for x in original.dependences:  # For deleting unwanted connection if needed
            name = x
            if temp.dependences.__contains__(x) is False:
                original.dependences.remove(x)
                for y in original.asEnd:
                    if y.fromComponent.alias == name:
                        original.asEnd.remove(y)

        if original.Ip != temp.Ip:
            original.Ip = temp.Ip
            self.ip_count()
            self.setAllIpColor()

        if original.endpoint != temp.endpoint:
            original.CheckItem.initializeComponent()

    def search_inside_list(self, the_list, name):
        flag = 0
        for x in the_list:
            if x.alias == name:
                flag += 1
                return True

        if flag == 0:
            return False

    def print_templ_settings(self):
        pass

    def add_component_templ(self):
        string = rcmanagerConfig.get_default_node()
        pos = self.CodeEditor.getCursorPosition()
        self.CodeEditor.insertAt(string, pos[0], pos[1])

    def search_entered_alias(self):  # Called when we type an alias and search it
        try:
            alias = self.lineEdit.text()
            if alias == '':
                raise Exception("No Name Entered")

            else:
                x = self.search_for_component(alias)  # Write here what ever should happen then
                self.graphTree.centerOn(x.graphicsItem)
            # x.graphicsItem.setSelected(True)
            self.Logger.logData(alias + "  Found")
        except Exception, e:
            self.Logger.logData("Search error::  " + str(e), "R")
        else:
            self.lineEdit.clear()
        finally:
            pass

    def have_changed(self):  # When the network have changed
        self.HadChanged = True

    def ip_count(self):  # To find all the computer present
        for x in self.componentList.__iter__():
            flag = False
            for y in self.ipList.__iter__():
                if y == x.Ip:
                    flag = True
                    break
            if not flag:
                self.ipList.append(x.Ip)

    def set_all_ip_color(self):  # A small algorithm to allot colors to each Ip
        try:
            diff = int(765 / self.ipList.__len__())
            for x in range(self.ipList.__len__()):
                num = (x + 1) * diff

                if num <= 255:
                    for y in self.componentList.__iter__():
                        if self.ipList[x] == y.Ip:
                            y.graphicsItem.IpColor = QtGui.QColor.fromRgb(num, 0, 0)
                elif 255 < num <= 510:
                    for y in self.componentList.__iter__():
                        if self.ipList[x] == y.Ip:
                            y.graphicsItem.IpColor = QtGui.QColor.fromRgb(255, num - 255, 0)
                elif num > 510:
                    for y in self.componentList.__iter__():
                        if self.ipList[x] == y.Ip:
                            y.graphicsItem.IpColor = QtGui.QColor.fromRgb(255, 255, num - 510)
            self.Logger.logData("IpColor allocated successfully")

        except Exception, e:
            raise Exception("Error during Ipcolors allocation " + str(e))

    def set_component_variables(self):  # Temperory function to be edited later
        for x in self.componentList.__iter__():
            x.View = self.graphTree
            x.mainWindow = self
            self.connect(x, QtCore.SIGNAL("networkChanged()"), self.have_changed)

    def set_directory_items(
            self):  # This will set and draw all the directory components+I have added the job of defining a connection in here
        for x in self.componentList.__iter__():
            x.DirectoryItem.setParent(self.scrollAreaWidgetContents)
            self.verticalLayout.insertWidget(self.verticalLayout.count() - 1, x.DirectoryItem)
            # print "Count is "+ str(self.verticalLayout.count())

    # TODO: implement or remove
    def component_settings(self, component):  # To edit the settings of currentComponent
        print "Settings of current component"

    # TODO: implement or remove
    def control_component(self, component):  # To open up the control panel of current component
        print "Controlling the current component"

    def down_selected_component(self):
        component = self.graphTree.CompoPopUpMenu.currentComponent
        rcmanagerConfig.down_component(component.parent, self.Logger)

    def up_selected_component(self):  # This will up a selected component
        component = self.graphTree.CompoPopUpMenu.currentComponent
        rcmanagerConfig.up_component(component.parent, self.Logger)

    def set_network_settings(self):  # To edit the network tree general settings
        print "network setting editing"

    def search_inside_tree(self):  # To search a particular component from tree
        print "Searching inside the tree"

    def up_all_components(self):  # To set all components in up position
        for x in self.componentList.__iter__():
            try:
                rcmanagerConfig.up_component(x, self.Logger)
            except Exception, e:
                pass

    def down_all_components(self):  # To set all components in down position
        for x in self.componentList.__iter__():
            try:
                rcmanagerConfig.down_component(x, self.Logger)
                time.sleep(.5)
            except Exception, e:
                pass

    def simulator_settings(self):  # To edit the simulatorSettings:Unfinished
        print "Simulator settings is on"

    def control_panel_settings(self):  # To edit the controlPanel Settings:Unfinshed
        print "Control panel settings"

    def editor_settings(self):  # To edit the editors settins:Unfinshed
        print "Editor Settings"

    def simulator_off(self):  # To switch Off the simulator::Unfiunished
        self.Logger.logData("Simulator ended")
        self.simulatorTimer.stop()

    def simulator_on(self):
        self.Logger.logData("Simulator started")
        self.simulatorTimer.start(300)

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

    def rcmanager_setting(self):  # To edit the setting of the entire rcmanager settings tool
        pass

    def exit_rcmanager(self):  # To exit the tool after doing all required process
        print "Exiting"

    def draw_all_components(self):  # Called to draw the components
        for x in self.componentList.__iter__():
            self.NetworkScene.addItem(x.graphicsItem)

    def draw_all_connection(self):  # This will start drawing Item
        for x in self.componentList.__iter__():
            for y in x.asBeg.__iter__():
                self.NetworkScene.addItem(y)

    def set_connection_items(
            self):  # This is called right after reading from a file,Sets all the connection graphicsItems
        for x in self.componentList.__iter__():

            for y in x.dependences.__iter__():
                try:
                    comp = self.search_for_component(y)
                    self.set_connection(comp, x)
                    self.Logger.logData("Connection from " + comp.alias + " to " + x.alias + " set")
                except Exception, e:
                    print "Error while setting connection ::" + str(e)

    def search_for_component(self, alias):  # this will search inside the components tree
        flag = False
        for x in self.componentList.__iter__():
            if x.alias == alias:
                flag = True
                return x
        if not flag:
            raise Exception("No such component with alias " + alias)

    def set_connection(self, toComponent, fromComponent):  # To set these two components
        connection = rcmanagerConfig.NodeConnection()

        connection.toComponent = toComponent
        connection.fromComponent = fromComponent

        fromComponent.asBeg.append(connection)
        toComponent.asEnd.append(connection)

        fromComponentPort, toComponentPort = rcmanagerConfig.findWhichPorts(fromComponent, toComponent)

        connection.fromX, connection.fromY = rcmanagerConfig.findPortPosition(fromComponent, fromComponentPort)
        connection.toX, connection.toY = rcmanagerConfig.findPortPosition(toComponent, toComponentPort)

    def remove_all_components(self):  # This removes all the components from everywhere#BUUUUUG
        len = self.componentList.__len__()
        for x in range(len):
            self.delete_component(self.componentList[len - 1 - x])

    def open_xml_file(self, terminalArg=False, UserHaveChoice=True):  # To open the xml files ::Unfinished
        rcmanagerConfig.NetworkValues()
        try:
            if self.HadChanged:  # To make sure the data we have been working on have been saved
                decision = self.SaveWarning.decide()
                if decision == "C":
                    raise Exception(" Reason:Canceled by User")
                elif decision == "S":
                    self.save_xml_file()
            if terminalArg is False and UserHaveChoice is True:
                self.filePath = QtGui.QFileDialog.getOpenFileName(self, 'Open file', initDir, '*.xml')

            string = rcmanagerConfig.get_string_from_file(self.filePath)
            self.CodeEditor.setText(string)
        except:
            self.Logger.logData("Couldn't read from file")
        self.refresh_tree_from_code(first_time=True)
        self.HadChanged = False

    def set_all_graphics_data(self):
        for x in self.componentList:
            x.setGraphicsData()

    def status_bar_file_name_write(self, string):
        label = QtGui.QLabel(string)
        self.statusbar.addWidget(label)

    def save_xml_file(self):  # To save the entire treesetting into a xml file
        try:
            save_file_name = QtGui.QFileDialog.getSaveFileName(self, 'Save File', initDir, '*.xml')
            save_file_name = str(save_file_name)
            if save_file_name.endswith(".xml") is False:
                save_file_name = save_file_name + ".xml"
            string = self.CodeEditor.text()
            try:
                xml_file = open(save_file_name, 'w')
                xml_file.write(string)
            except:
                raise Exception("Can't Open" + save_file_name)
            # rcmanagerConfig.writeToFile(file,string)

            self.Logger.logData("Saved to file " + save_file_name + " ::successful")
        except Exception, e:
            self.Logger.logData("Saving to file" + save_file_name + " ::failed " + str(e), "R")

    # def saveTofile(fileName):#Save to this filename
    #	rcmanagerConfig.writeConfigToFile(self.networkSettings,self.componentList,fileName)

    def set_zoom(self):  # To connect the slider motion to zooming
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

    def add_new_node(self):  # Called where right clicked and seleceted to add a new node
        pos = self.graphTree.BackPopUpMenu.pos
        scene_pos = self.graphTree.mapToScene(pos)
        self.add_new_component(scene_pos)

    # The final function which takes care of adding new component default zero
    def add_new_component(self, pos=QtCore.QPointF()):
        component = rcmanagerConfig.CompInfo(view=self.graphTree, mainWindow=self,
                                             name="Component" + str(self.componentList.__len__()))
        # component.CheckItem.setLogger(self.Logger)
        self.componentList.append(component)
        component.x = pos.x()
        component.y = pos.y()
        component.graphicsItem.setPos(pos)
        self.NetworkScene.addItem(component.graphicsItem)
        self.refresh_code_from_tree()
        component.DirectoryItem.setParent(self.scrollAreaWidgetContents)
        self.verticalLayout.insertWidget(self.verticalLayout.count() - 1, component.DirectoryItem)
        self.tabWidget.setCurrentIndex(1)
        self.CodeEditor.findFirst("Component" + str(self.componentList.__len__() - 1), False, True, True, True)

    def delete_component(self, component):  # This will delete the component Not completed

        #	print component.alias

        for x in component.asBeg:
            x.toComponent.dependences.remove(component.alias)
            x.toComponent.asEnd.remove(x)
            self.NetworkScene.removeItem(x)
        for x in component.asEnd:
            x.fromComponent.asBeg.remove(x)
            self.NetworkScene.removeItem(x)

        if component.group is not None:
            component.group.remove_component(component)
        component.CheckItem.stop()
        self.NetworkScene.removeItem(component.graphicsItem)
        self.NetworkScene.update()
        component.DirectoryItem.hide()

        for x in self.componentList.__iter__():
            if component.graphicsItem == x.graphicsItem:
                self.componentList.remove(component)
        self.refresh_code_from_tree()
        self.Logger.logData("Deleted the component '" + component.alias + "' successfully")

    # print self.componentList.__len__()

    def keyPressEvent(self, event):
        if event.key() == Qt.Qt.Key_F5:
            self.refresh_tree_from_code()
        elif event.key() == Qt.Qt.Key_F11:
            self.actionFull_Screen.toggle()
            self.toggle_full_screen_view()


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MainClass()
    window.show()

    if sys.argv.__len__() > 1:
        try:
            if sys.argv.__len__() > 3:
                raise Exception("Only two args allowed:: Eg\n rcmanager Filename Logfilename")

            if sys.argv.__len__() > 2:
                if sys.argv[2].endswith(".log"):
                    window.Logger.setFile(sys.argv[2])
                else:
                    raise Exception("The log file should end with .log")

            if sys.argv[1].endswith(".xml"):
                window.filePath = sys.argv[1]
                window.open_xml_file(terminalArg=True)

            elif sys.argv[1].endswith(".log"):
                window.Logger.setFile(sys.argv[1])
            else:
                raise Exception("The target should be an .xml file or log File should be .log file")

        except Exception, e:
            print "Errorrr ::" + str(e)
            sys.exit()

    try:
        QtCore.QTimer.singleShot(50, window.center_align_graph)
        ret = app.exec_()

    except:
        print 'Some error happened.'

    sys.exit()
