
from PyQt4 import QtCore

class rcmanagerSignals(QtCore.QObject):

	sample = QtCore.pyqtSignal()
   	init = QtCore.pyqtSignal(str)
   	addNode = QtCore.pyqtSignal(dict)

	def __init__(self, parent=None):
		QtCore.QObject.__init__(self)
