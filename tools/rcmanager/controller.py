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

import threading
import xmlreader

from logger import RCManagerLogger
from PyQt4.QtGui import QColor
from rcmanagerSignals import CustomSignalCollection

class Controller():
    """This is the Controller object for our MVC model. It connects the Model
    and the view, by reacting to the signals emitted by the view and
    making the necessary changes to the Model"""

    def __init__(self, model, view):
        self._logger = RCManagerLogger().get_logger("RCManager.Controller")

        self.need_to_save = False
        self.view = view
        self.model = model

        self.isModelReady = False
        self.isViewReady = False
        self.isControllerReady = False

    def model_init_action(self):
        self.isModelReady = True
        self._logger.info("Model object initialized")

    def view_init_action(self):
        self.isViewReady = True
        self._logger.info("View object initialized")

    def controller_init_action(self, filename, isNewFile=True):
        self.isControllerReady = True
        self._logger.info("Controller object initialized")

        # save the filename for future use
        if isNewFile:
            self.view.filename = filename
            self.view.dirtyBit = False

        # read the xml data from the file
        self.xml = xmlreader.get_text_from_file(str(filename))

        # check the xml data for formatting issues
        if not xmlreader.validate_xml(self.xml):
            self._logger.error("XML validation failed. Please use a correctly formatted XML file")
            return

        self.model.load_from_xml(self.xml)
        self.load_model_into_viewer(self.xml)
        self.configure_viewer()

    # configure the user interface
    def configure_viewer(self):
        if 'backgroundColor' in self.model.generalInformation.keys():
            color = QColor(self.model.generalInformation['backgroundColor']['@value'])
            self.view.set_background_color(color)

    # component related tasks
    def start_component(self):
        selectedNodes = self.view.graph_visualization.selected_nodes()
        for node in selectedNodes:
            self.model.execute_start_command(str(node))

    def stop_component(self):
        selectedNodes = self.view.graph_visualization.selected_nodes()
        for node in selectedNodes:
            self.model.execute_stop_command(str(node))

    def add_component(self, nodedata):
        self.model.add_node(nodedata)

    def remove_component(self):
        selectedNodes = self.view.graph_visualization.selected_nodes()
        for node in selectedNodes:
            self.view.remove_node(str(node))
            self.model.remove_node(str(node))        

    # XML related tasks 
    def load_model_into_viewer(self, xml):
        self.view.clear_graph_visualization()
        self.view.set_editor_text(xml)
        self.view.dirtyBit = False

        # adding nodes
        if self.view:
            for node, data in self.model.graph.nodes_iter(data=True):
                try:
                    xpos = float(data['xpos']['@value'])
                    ypos = float(data['ypos']['@value'])
                    position = (xpos, ypos)
                    self.view.add_node(node, data, position)
                except Exception, e:
                    self._logger.error("Node postion value for " + node + " are incorrect")
                    self.view.add_node(node, data)
            for orig, dest, data in self.model.graph.edges_iter(data=True):
                self.view.add_edge(orig, dest, data)
        else:
            raise Exception("A view must exist to update from model")

    def update_model(self):
        currentNodePosition = self.view.get_graph_nodes_positions()
        for i in currentNodePosition:
            xpos, ypos = currentNodePosition[i]

            new = {'@value': str(xpos)}
            old = self.model.graph.node[str(i)].get('xpos')
            if not old == new:
                self.view.dirtyBit = True
            self.model.graph.node[str(i)]['xpos'] = new

            new = {'@value': str(ypos)}
            old = self.model.graph.node[str(i)].get('ypos')
            if not old == new:
                self.view.dirtyBit = True
            self.model.graph.node[str(i)]['ypos'] = new

        color = self.view.graph_visualization.background_color
        new = {'@value': color.name()}
        old = self.model.generalInformation.get('backgroundColor')

        # this statement makes sure that #FF0000 is perceived to be the same as #ff0000
        if old is not None:
            old['@value'] = str(old['@value']).lower()
        if not old == new:
            self.view.dirtyBit = True
        self.model.generalInformation['backgroundColor'] = {'@value': color.name()}

    def normalise_dict(self, d):
        """
        Recursively convert dict-like object (eg OrderedDict) into plain dict.
        Sorts list values.
        """
        out = {}
        for k, v in dict(d).iteritems():
            if hasattr(v, 'iteritems'):
                out[k] = self.normalise_dict(v)
            elif isinstance(v, list):
                out[k] = []
                for item in sorted(v):
                    if hasattr(item, 'iteritems'):
                        out[k].append(self.normalise_dict(item))
                    else:
                        out[k].append(item)
            else:
                out[k] = v
        return out

    # load/save operations 
    def load_manager_file(self, filename, isNewFile=True):
        self.controller_init_action(filename, isNewFile)

    def save_manager_file(self, filename):
        try:
            self.update_model()
            self.model.export_xml_to_file(str(filename))
        except Exception, e:
            self._logger.error("Couldn't save to file " + filename)
            raise e

    def check_dirty_bit(self):
        index = self.view.tabWidget.currentIndex()
        if index == 0:
            self.update_model()
        elif index == 1:
            try:
                first = self.normalise_dict(xmlreader.read_from_text(str(self.xml), 'xml'))
                second = self.normalise_dict(xmlreader.read_from_text(str(self.view.codeEditor.text()), 'xml'))

                if not first == second:
                    self.view.dirtyBit = True
            except Exception, e:
                self._logger.error("XML file in code editor is incorrectly formatted")
