
from PyQt4 import QtCore

class rcmanagerSignals(QtCore.QObject):

	sample = QtCore.pyqtSignal()
   	modelIsReady = QtCore.pyqtSignal()
   	viewerIsReady = QtCore.pyqtSignal()
   	controllerIsReady = QtCore.pyqtSignal()
   	addNode = QtCore.pyqtSignal(dict)

	def __init__(self, parent=None):
		QtCore.QObject.__init__(self)
