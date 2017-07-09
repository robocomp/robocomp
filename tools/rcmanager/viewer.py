from PyQt4 import QtCore, QtGui, uic

MainWindow = uic.loadUiType("formManager.ui")[0]  # Load the UI

class Viewer(QtGui.QMainWindow, MainWindow):
	"""docstring for Viewer"""

	def __init__(self, arg=None):
		print "------------------------------------"
		print "Hello this is Viewer coming up"
		super(Viewer, self).__init__(arg)
		self.setupUi(self)
		self.show()


