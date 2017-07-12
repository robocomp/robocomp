
from PyQt4 import QtCore

class rcmanager_signals(QtCore.QObject):

   	sample = QtCore.pyqtSignal()

	def __init__(self, parent=None):
		QtCore.QObject.__init__(self)
