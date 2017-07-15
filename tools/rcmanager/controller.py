
from logger import RCManagerLogger
from PyQt4 import QtCore
from PyQt4.QtCore import QObject, pyqtSignal, pyqtSlot
# import pdb

class Controller():
    """This is the Controller object for our MVC model. It connects the Model
    and the Viewer, by reacting to the signals emitted by the Viewer and
    making the necessary changes to the Model"""
    
    def __init__(self, rcmanagerSignals):
        # self._logger = RCManagerLogger().get_logger("RCManager.Controller")
        # self._logger.info("Hello, this is Controller coming up")

        self.need_to_save = False
        self.view = None
        self.model = None
        self.rcmanagerSignals = rcmanagerSignals
        
        self.isModelReady = False
        self.isViewerReady = False
        self.isControllerReady = False
        
       	self.signal_connections()
       	
        pass

    def signal_connections(self):
    	self.rcmanagerSignals.init.connect(self.init_action)
    	
    def init_action(self, string):
    	print string, "object initialized"
    	
    	if string == 'Model':
    		self.isModelReady = True
    	elif string == 'Viewer':
     		self.isViewerReady = True
     	elif string == 'Controller':
     		self.isControllerReady = True
     		self.refresh_graph_from_model()
     	
    def refresh_graph_from_model(self):
		# adding nodes
		for node, data in self.model.graph.nodes_iter(data=True):
			print "The controller sent signal to draw component:", data['@alias']
			self.rcmanagerSignals.addNode.emit(data)
	
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
