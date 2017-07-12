
from logger import RCManagerLogger
from PyQt4 import QtCore
from PyQt4.QtCore import QObject, pyqtSignal, pyqtSlot


class Controller():
    """This is the Controller object for our MVC model. It connects the Model
    and the Viewer, by reacting to the signals emitted by the Viewer and
    making the necessary changes to the Model"""
    
    def __init__(self, model, viewer, rcmanager_signals):
        self._logger = RCManagerLogger().get_logger("RCManager.Controller")
        # self._logger.info("Hello, this is Controller coming up")
        # we set here the signal/slots connections to let data flow

        self.need_to_save = False
        self.view = viewer
        self.model = model
        self.rcmanager_signals = rcmanager_signals
        
        # reacting to a specific signal 
        self.rcmanager_signals.sample.connect(self.sample_action)
        pass

    def signal_connections(self):
        pass
        
    def sample_action(self):
		print "hello, this is printed because a sample signal was emitted"

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
