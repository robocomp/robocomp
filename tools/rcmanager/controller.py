
from logger import RCManagerLogger
from PyQt4 import QtCore
from PyQt4.QtCore import QObject, pyqtSignal, pyqtSlot
from xml.etree import ElementTree

class Controller():
    """This is the Controller object for our MVC model. It connects the Model
    and the view, by reacting to the signals emitted by the view and
    making the necessary changes to the Model"""

    def __init__(self, model, view, rcmanagerSignals):
        self._logger = RCManagerLogger().get_logger("RCManager.Controller")

        self.need_to_save = False
        self.view = view
        self.model = model
        self.rcmanagerSignals = rcmanagerSignals

        self.isModelReady = False
        self.isViewReady = False
        self.isControllerReady = False

        self.signal_connections()
        pass

    def signal_connections(self):
        pass

    def model_init_action(self):
        self.isModelReady = True
        self._logger.info("Model object initialized")

    def view_init_action(self):
        self.isViewReady = True
        self._logger.info("view object initialized")

    def controller_init_action(self, filename):
        self.isControllerReady = True
        self._logger.info("Controller object initialized")

        # Save the filename for future use
        self.view.filename = filename

        # Read the xml data from the file
        file = open(filename, 'r')
        xml = file.read()

        # Check the xml data for formatting issues
        try:
            ElementTree.fromstring(xml)
        except Exception, e:
            self._logger.error("XML validation failed. Please use a correctly formatted XML file")
            return

        # Model uses filename as the input
        self.model.load_from_xml(filename)

        # The following function requires the xml string as the input
        self.refresh_graph_from_model(xml)

    def start_component(self, componentAlias):
        self.model.execute_start_command(str(componentAlias))

    def stop_component(self, componentAlias):
        self.model.execute_stop_command(str(componentAlias))

    def refresh_graph_from_model(self, xml):
        self.view.clear_graph_visualization()
        self.view.set_editor_text(xml)
        # adding nodes
        if self.view:
            for node, data in self.model.graph.nodes_iter(data=True):
                # print "The controller sent signal to draw component:", data['@alias']
                # self.rcmanagerSignals.addNode.emit(data)

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
            self.model.graph.node[str(i)]['xpos']['@value'] = str(xpos)
            self.model.graph.node[str(i)]['ypos']['@value'] = str(ypos)

    def load_manager_file(self, filename):
        """
        try:
            if self.need_to_save:  # To make sure the data we have been working on have been saved
                decision = self.view.save_warning.decide()
                if decision == "C":
                    raise Exception("Reason: Canceled by User")
                elif decision == "S":
                    self.save_manager_file()
            if terminalArg is False and UserHaveChoice is True:
                self.filePath = self.view.open_file_dialog()

            string = self.model.get_string_from_file(self.filePath)
            self.CodeEditor.setText(string)
        except:
            self._logger.error("Couldn't read from file")
        self.view.refresh_tree_from_code(first_time=True)
        self.need_to_save = False
        """
        self.controller_init_action(filename)

    def save_manager_file(self, filename):
        try:
            self.update_model()
            self.model.export_xml_to_file(str(filename))
        except Exception, e:
            self._logger.error("Couldn't save to file " + filename)
            
