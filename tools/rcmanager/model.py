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

import os
import Ice
import sys
import time
import threading
import xmlreader
import networkx as nx

from PyQt4 import QtCore
from logger import RCManagerLogger
from yakuake_support import ProcessHandler
from rcmanagerSignals import CustomSignalCollection

global_ic = Ice.initialize()

class ComponentChecker(threading.Thread):
    def __init__(self, componentAlias, endpoint):
        threading.Thread.__init__(self)
        self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
        self.daemon = True
        self.reset()
        self.exit = False
        self.alive = False
        self.aPrx = None
        self.componentAlias = componentAlias
        try:
            self.aPrx = global_ic.stringToProxy(endpoint)
            self.aPrx.ice_timeout(1)
        except:
            print "Error creating proxy to " + endpoint
            if len(endpoint) == 0:
                print 'Error - No endpoint specified'
            raise

    def run(self):
        global global_ic
        while self.exit == False:
            previousAliveValue = self.alive
            try:
                self.mutex.lock()
                self.alive = True
                self.mutex.unlock()
                self.aPrx.ice_ping()
            except (Ice.ConnectionRefusedException, Ice.ConnectFailedException) as e:
                self.mutex.lock()
                self.alive = False
                self.mutex.unlock()
            if previousAliveValue != self.alive:
                if self.alive:
                    CustomSignalCollection.componentRunning.emit(self.componentAlias)
                else:
                    CustomSignalCollection.componentStopped.emit(self.componentAlias)

            time.sleep(0.5)

    def reset(self):
        self.mutex.lock()
        self.alive = False
        self.mutex.unlock()

    def is_alive(self):
        self.mutex.lock()
        r = self.alive
        self.mutex.unlock()
        return r

    def stop(self):
        self.exit = True

    def begin(self):
        if not self.isAlive(): self.start()

class Model():
    """This is the Model object for our MVC model. It stores the component
    graph and contains the functions needed to manipulate it."""

    def __init__(self):
        self._logger = RCManagerLogger().get_logger("RCManager.Model")

        # this is the networkx digraph object
        self.graph = nx.DiGraph()

        # this dictionary stores the component checker threads for each component
        self.componentChecker = dict()

        # this dictionary stores the general configuration informtation about the viewer
        self.generalInformation = dict()

        # this is the process handler for the model
        self.processHandler = ProcessHandler()

    def load_from_xml(self, xml):
        # we go through the dictionary to create the graph
        # we have "rcmanager" and "nodes" keys

        # Clear previous data
        self.graph.clear()
        self.graph.node.clear()

        # Convert the xml data into python dict format
        xmldata = xmlreader.read_from_text(xml, 'xml')

        # xmldata['rcmanager']['node'] should be a list of items,
        # even if there is only one node in the xml
        if not isinstance(xmldata['rcmanager']['node'], list):
            xmldata['rcmanager']['node'] = [xmldata['rcmanager']['node']]

        # creating nodes
        # the try catch block is added to handle cases
        # when the xml document contains no nodes
        try:
            for k in xmldata['rcmanager']['node']:
                self.add_node(k)
        except Exception, e:
            raise e

        # creating edges
        # the try catch block is added to handle cases
        # when the xml document contains no dependencies / edges
        try:
            for i in xmldata['rcmanager']['node']:
                if 'dependence' not in i.keys():
                    continue

                if not isinstance(i['dependence'], list):
                    i['dependence'] = [i['dependence']]

                for j in i['dependence']:
                    if (i['@alias'] in self.graph.node.keys()) and (j['@alias'] in self.graph.node.keys()):
                        self.add_edge(i['@alias'], j['@alias'])
        except Exception, e:
            raise e

        try:
            for key, value in xmldata['rcmanager']['generalInformation'].items():
                self.generalInformation[key] = value
        except KeyError:
            pass
        except Exception, e:
            raise e

        CustomSignalCollection.modelIsReady.emit()

    def add_node(self, nodedata):
        self.graph.add_node(nodedata['@alias'])
        self.componentChecker[nodedata['@alias']] = ComponentChecker(str(nodedata['@alias']),
                                                                     str(nodedata['@endpoint']))
        self.componentChecker[nodedata['@alias']].begin()
        for key, value in nodedata.items():
            self.graph.node[nodedata['@alias']][key] = value

    def remove_node(self, node):
        node = str(node)
        if node in self.graph.nodes():
            self.graph.remove_node(node)

    def add_edge(self, fromNode, toNode):
        self.graph.add_edge(fromNode, toNode)

    def get_component_running_status(self, componentAlias):
        return self.componentChecker[componentAlias].is_alive()

    # this functions executes the command for starting a component
    def execute_start_command(self, componentAlias):
        if not self.get_component_running_status(componentAlias):
            tabTitle, processId = self.processHandler.start_process_in_existing_session(componentAlias, \
                                                                                        self.graph.node[componentAlias]['upCommand']['@command'])
            self._logger.info("Component: " + componentAlias + " started in tab: " + tabTitle + " with PID: " + processId)
        else:
            self._logger.debug("Component: " + componentAlias + " is already running")

    # this functions executes the command for killing a component
    def execute_stop_command(self, componentAlias):
        if not self.get_component_running_status(componentAlias):
            self._logger.debug("Component: " + componentAlias + " is not running")
        else:
            self.processHandler.stop_process_in_session(componentAlias)
            self._logger.info("Component: " + componentAlias + " stopped")

    # this function writes the xml data to the given filename
    def export_xml_to_file(self, filename):
        open(filename, 'w').close()
        fileDescriptor = open(filename, 'a')

        line = '<?xml version="1.0" encoding="UTF-8"?>\n\n'
        line += '<rcmanager>\n'
        fileDescriptor.write(line + '\n')

        line = self.dict_to_xml(self.generalInformation, 'generalInformation', 1)
        fileDescriptor.write(line + '\n\n')

        for i in self.graph.node.keys():
            line = self.dict_to_xml(self.graph.node[i], 'node', 1)
            fileDescriptor.write(line + '\n\n')

        line = '</rcmanager>'
        fileDescriptor.write(line + '\n')

        fileDescriptor.close()

    # this function converts the graph dict to the standard xml format used by rcmanager
    def dict_to_xml(self, dictionary, tagname, indentSpaceCount):
        line = (' ' * indentSpaceCount) + '<' + tagname

        subtags = list()

        for i in dictionary.keys():
            if not isinstance(dictionary[i], list):
                items = [dictionary[i]]
            else:
                items = dictionary[i]

            for j in items:
                if isinstance(j, dict):
                    subtags.append(self.dict_to_xml(j, i, (indentSpaceCount + 1)))
                else:
                    # eliminate the '@' symbol in the front
                    attr = i[:0] + i[(1):]
                    line = line + ' ' + attr + '="' + j + '"'

        if len(subtags) == 0:
            line = line + '/>'
        else:
            line = line + '>\n'
            for i in subtags:
                line = line + i + '\n'
            line = line + (' ' * indentSpaceCount) + '</' + tagname + '>'

        return line

    # this functions emits a sample signal
    def sample_emit(self):
        print "sample signal was emitted"
        CustomSignalCollection.sample.emit('Model')

if __name__ == '__main__':
    # sample test case to see the working of the Model class  
    model = Model()
    
    model.add_node({'@alias': 'A'})
    model.add_node({'@alias': 'B'})
    model.add_node({'@alias': 'C'})
    model.add_node({'@alias': 'D'})
    
    model.add_edge('A', 'B')
    model.add_edge('B', 'C')
    model.add_edge('B', 'D')
    model.add_edge('C', 'D')
    model.add_edge('D', 'B')
    
    print "Number of nodes:", model.graph.number_of_nodes()
    print "Number of edges:", model.graph.number_of_edges()
    print "Adjacencies: ", model.graph.adj
