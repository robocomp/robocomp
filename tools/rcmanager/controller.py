
from logger import RCManagerLogger
from PyQt4 import QtCore
from PyQt4.QtCore import QObject, pyqtSignal, pyqtSlot

class Controller():
    """This is the Controller object for our MVC model. It connects the Model
    and the Viewer, by reacting to the signals emitted by the Viewer and
    making the necessary changes to the Model"""

    def __init__(self, model, viewer, rcmanagerSignals):
        # self._logger = RCManagerLogger().get_logger("RCManager.Controller")
        # self._logger.info("Hello, this is Controller coming up")

        self.need_to_save = False
        self.viewer = viewer
        self.model = model
        self.rcmanagerSignals = rcmanagerSignals
        
        self.isModelReady = False
        self.isViewerReady = False
        self.isControllerReady = False
        
       	self.signal_connections()
        pass

    def signal_connections(self):
    	pass
    	
    def model_init_action(self):
    	self.isModelReady = True
    	print "Model object initialized"
    	
    def viewer_init_action(self):
		self.isViewerReady = True
		print "Viewer object initialized"
    
    def controller_init_action(self):
    	self.isControllerReady = True
    	print "Controller object initialized"
    	self.refresh_graph_from_model()
     	
    def refresh_graph_from_model(self):
        # adding nodes
        if self.view:
            for node, data in self.model.graph.nodes_iter(data=True):
                # print "The controller sent signal to draw component:", data['@alias']
                # self.rcmanagerSignals.addNode.emit(data)
                self.view.add_node(node, data)
            for orig, dest, data in self.model.graph.edges_iter(data=True):
                self.view.add_edge(orig, dest, data)
        else:
            raise Exception("A view must exist to update from model")
	
    def load_manager_file(self, terminalArg=False, UserHaveChoice=True):  # To open the xml files ::Unfinished
        try:
            if self.need_to_save:  # To make sure the data we have been working on have been saved
                decision = self.view.save_warning.decide()
                if decision == "C":
                    raise Exception("Reason: Canceled by User")
                elif decision == "S":
                    self.save_xml_file()
            if terminalArg is False and UserHaveChoice is True:
                self.filePath = self.view.open_file_dialog()

            string = self.model.get_string_from_file(self.filePath)
            self.CodeEditor.setText(string)
        except:
            self._logger.error("Couldn't read from file")
        self.view.refresh_tree_from_code(first_time=True)
        self.need_to_save = False
