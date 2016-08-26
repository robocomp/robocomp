#!/usr/bin/env python2.7
#
#  -----------------------
#  ----- rcmanager -----
#  -----------------------
#  An ICE component manager.
#

#    Copyright (C) 2008-2010 by RoboLab - University of Extremadura
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
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#



#
# CODE BEGINS
#
import sys, time, traceback, os

import Ice

from PyQt4 import QtCore, QtGui, Qt
from ui_editorForm import Ui_Form

import rcmanagerConfig

configFile = os.path.expanduser('~/.rcmanager')

#
# Application main class.
#
class rcmanagerEditorWidget(QtGui.QDialog):
	def __init__(self):
		self.compConfig = []


		# Gui config
		QtGui.QDialog.__init__(self)
		self.ui = Ui_Form()
		self.ui.setupUi(self)

		self.resetGui()

		# Set connections
		self.connect(self.ui.table, QtCore.SIGNAL("cellClicked(int, int)"), self.selectComponent)
		self.connect(self.ui.pathButton, QtCore.SIGNAL("clicked()"), self.browsePath)
		self.connect(self.ui.newButton, QtCore.SIGNAL("clicked()"), self.newRow)
		self.connect(self.ui.deleteButton, QtCore.SIGNAL("clicked()"), self.delete)
		self.connect(self.ui.readButton, QtCore.SIGNAL("clicked()"), self.read)
		self.connect(self.ui.writeButton, QtCore.SIGNAL("clicked()"), self.write)
		#aliasLine pathLine upLine downLine dependencesLine configLine endpointLine
		self.connect(self.ui.aliasLine, QtCore.SIGNAL("textChanged(QString)"), self.modifyAlias)
		self.connect(self.ui.pathLine, QtCore.SIGNAL("textChanged(QString)"), self.modifyPath)
		self.connect(self.ui.upLine, QtCore.SIGNAL("textChanged(QString)"), self.modifyUp)
		self.connect(self.ui.downLine, QtCore.SIGNAL("textChanged(QString)"), self.modifyDown)
		self.connect(self.ui.dependencesLine, QtCore.SIGNAL("textChanged(QString)"), self.modifyDependences)
		self.connect(self.ui.configLine, QtCore.SIGNAL("textChanged(QString)"), self.modifyConfig)
		self.connect(self.ui.endpointLine, QtCore.SIGNAL("textChanged(QString)"), self.modifyEndpoint)
		#self.readConfig(configFile)
	def resetGui(self):
		for i in range(self.ui.table.columnCount()):
			self.ui.table.setColumnWidth(i, 100)
		self.ui.table.setColumnWidth(1, 122)
		self.ui.table.setColumnWidth(2, 122)

		self.ui.table.clear()
		headerItem = QtGui.QTableWidgetItem()
		headerItem.setText(QtGui.QApplication.translate("Form", "Alias", None, QtGui.QApplication.UnicodeUTF8))
		self.ui.table.setHorizontalHeaderItem(0,headerItem)
		headerItem1 = QtGui.QTableWidgetItem()
		headerItem1.setText(QtGui.QApplication.translate("Form", "Deps", None, QtGui.QApplication.UnicodeUTF8))
		self.ui.table.setHorizontalHeaderItem(1,headerItem1)
		headerItem2 = QtGui.QTableWidgetItem()
		headerItem2.setText(QtGui.QApplication.translate("Form", "Endpoint", None, QtGui.QApplication.UnicodeUTF8))
		self.ui.table.setHorizontalHeaderItem(2,headerItem2)
		headerItem3 = QtGui.QTableWidgetItem()
		headerItem3.setText(QtGui.QApplication.translate("Form", "Path", None, QtGui.QApplication.UnicodeUTF8))
		self.ui.table.setHorizontalHeaderItem(3,headerItem3)
		headerItem4 = QtGui.QTableWidgetItem()
		headerItem4.setText(QtGui.QApplication.translate("Form", "Up Command", None, QtGui.QApplication.UnicodeUTF8))
		self.ui.table.setHorizontalHeaderItem(4,headerItem4)
		headerItem5 = QtGui.QTableWidgetItem()
		headerItem5.setText(QtGui.QApplication.translate("Form", "Down Command", None, QtGui.QApplication.UnicodeUTF8))
		self.ui.table.setHorizontalHeaderItem(5,headerItem5)
		headerItem6 = QtGui.QTableWidgetItem()
		headerItem6.setText(QtGui.QApplication.translate("Form", "Config", None, QtGui.QApplication.UnicodeUTF8))
		self.ui.table.setHorizontalHeaderItem(6,headerItem6)

		for i in range(self.ui.table.columnCount()):
			item = QtGui.QTableWidgetItem()
			item.setText('')
			item.setFlags(QtCore.Qt.ItemIsSelectable)
			self.ui.table.setItem(self.ui.table.rowCount() - 1, i, item)

	def read(self):
		s = QtGui.QFileDialog.getOpenFileName (self, "Open file", os.environ["HOME"], "*.xml")
		print s
		self.readConfig(s)
	def write(self):
		global dict
		s = QtGui.QFileDialog.getSaveFileName (self, "Save file", os.environ["HOME"], "*.xml")
		self.compConfig = []

		print self.ui.table.rowCount()
		for row in range(self.ui.table.rowCount()):
			comp = rcmanagerConfig.CompInfo()
			comp.alias = str(self.ui.table.item(row, 0).text())
			comp.dependences = []
			l = str(self.ui.table.item(row, 1).text()).split(',')
			for dep in l:
				if len(dep) > 0:
					comp.dependences.append(dep)
			comp.endpoint = str(self.ui.table.item(row, 2).text())
			comp.workingdir = str(self.ui.table.item(row, 3).text())
			comp.compup = str(self.ui.table.item(row, 4).text())
			comp.compdown = str(self.ui.table.item(row, 5).text())
			comp.configFile = str(self.ui.table.item(row, 6).text())
			self.compConfig.append(comp)

		rcmanagerConfig.writeConfigToFile(self.dict, self.compConfig, s)

	def readConfig(self, filePath):
		self.compConfig, self.dict = rcmanagerConfig.getConfigFromFile(filePath)

		i = 0
		self.ui.table.setSortingEnabled(False)
		for listItem in self.compConfig:
			self.newRow()
			
			item = self.ui.table.item(i, 0)
			item.setText(listItem.alias)
			item.setFlags(QtCore.Qt.ItemIsSelectable)
			self.ui.table.setItem(i, 0, item)

			item = self.ui.table.item(i, 1)
			item.setText(','.join(listItem.dependences))
			item.setFlags(QtCore.Qt.ItemIsSelectable)
			self.ui.table.setItem(i, 1, item)

			item = self.ui.table.item(i, 2)
			item.setText(listItem.endpoint)
			item.setFlags(QtCore.Qt.ItemIsSelectable)
			self.ui.table.setItem(i, 2, item)

			item = self.ui.table.item(i, 3)
			item.setText(listItem.workingdir)
			item.setFlags(QtCore.Qt.ItemIsSelectable)
			self.ui.table.setItem(i, 3, item)

			item = self.ui.table.item(i, 4)
			item.setText(listItem.compup)
			item.setFlags(QtCore.Qt.ItemIsSelectable)
			self.ui.table.setItem(i, 4, item)

			item = self.ui.table.item(i, 5)
			item.setText(listItem.compdown)
			item.setFlags(QtCore.Qt.ItemIsSelectable)
			self.ui.table.setItem(i, 5, item)

			item = self.ui.table.item(i, 6)
			item.setText(listItem.configFile)
			item.setFlags(QtCore.Qt.ItemIsSelectable)
			self.ui.table.setItem(i, 6, item)

			i+=1
		self.ui.table.setSortingEnabled(True)

	@QtCore.pyqtSignature("int, int")
	def selectComponent(self, x, y):
		self.ui.aliasLine.setText(QtCore.QString(self.ui.table.item(x, 0).text()))
		self.ui.endpointLine.setText(QtCore.QString(self.ui.table.item(x, 2).text()))
		self.ui.dependencesLine.setText(QtCore.QString(self.ui.table.item(x, 1).text()))
		self.ui.pathLine.setText(QtCore.QString(self.ui.table.item(x, 3).text()))
		self.ui.upLine.setText(QtCore.QString(self.ui.table.item(x, 4).text()))
		self.ui.downLine.setText(QtCore.QString(self.ui.table.item(x, 5).text()))
		self.ui.configLine.setText(QtCore.QString(self.ui.table.item(x, 6).text()))

	def browsePath(self):
		s = QtGui.QFileDialog.getExistingDirectory(self, "Select directory", os.environ["HOME"])
		if s:
			self.ui.pathLine.setText(s)

	#
	# Clears the interface selection when the user presses 'Esc'
	def keyPressEvent(self, keyevent):
		pass
		#if keyevent.key() == 0x01000000:
			#self.ui.checkList.clearSelection()
	def newRow(self):
		self.ui.table.setRowCount(self.ui.table.rowCount() + 1)

		self.ui.table.setVerticalHeaderItem(self.ui.table.rowCount()-1, QtGui.QTableWidgetItem())
		for i in range(self.ui.table.columnCount()):
			item = QtGui.QTableWidgetItem()
			item.setFlags(QtCore.Qt.ItemIsSelectable)
			item.setText('')
			self.ui.table.setItem(self.ui.table.rowCount() - 1, i, item)
			self.ui.table.setCurrentCell(self.ui.table.rowCount() - 1, 0)

	def modifyAlias(self):
		row = self.ui.table.currentRow()
		self.ui.table.setSortingEnabled(False)
		item = self.ui.table.item(row, 0)
		item.setText(self.ui.aliasLine.text())
		self.ui.table.setItem(row, 0, item)
		self.ui.table.setSortingEnabled(True)
	def modifyDependences(self):
		row = self.ui.table.currentRow()
		self.ui.table.setSortingEnabled(False)
		item = self.ui.table.item(row, 1)
		item.setText(self.ui.dependencesLine.text())
		self.ui.table.setItem(row, 1, item)
		self.ui.table.setSortingEnabled(True)
	def modifyEndpoint(self):
		row = self.ui.table.currentRow()
		self.ui.table.setSortingEnabled(False)
		item = self.ui.table.item(row, 2)
		item.setText(self.ui.endpointLine.text())
		self.ui.table.setItem(row, 2, item)
		self.ui.table.setSortingEnabled(True)
	def modifyPath(self):
		row = self.ui.table.currentRow()
		self.ui.table.setSortingEnabled(False)
		item = self.ui.table.item(row, 3)
		item.setText(self.ui.pathLine.text())
		self.ui.table.setItem(row, 3, item)
		self.ui.table.setSortingEnabled(True)
	def modifyUp(self):
		row = self.ui.table.currentRow()
		self.ui.table.setSortingEnabled(False)
		item = self.ui.table.item(row, 4)
		item.setText(self.ui.upLine.text())
		self.ui.table.setItem(row, 4, item)
		self.ui.table.setSortingEnabled(True)
	def modifyDown(self):
		row = self.ui.table.currentRow()
		self.ui.table.setSortingEnabled(False)
		item = self.ui.table.item(row, 5)
		item.setText(self.ui.downLine.text())
		self.ui.table.setItem(row, 5, item)
		self.ui.table.setSortingEnabled(True)
	def modifyConfig(self):
		row = self.ui.table.currentRow()
		self.ui.table.setSortingEnabled(False)
		item = self.ui.table.item(row, 6)
		item.setText(self.ui.configLine.text())
		self.ui.table.setItem(row, 6, item)
		self.ui.table.setSortingEnabled(True)
	def delete(self):
		self.ui.table.removeRow(self.ui.table.currentRow())
	def closeEvent(self, e):
		self.emit(QtCore.SIGNAL("finished()"))

# Create the Qt application, the class, and runs the program
#
if __name__ == '__main__':
	app = QtGui.QApplication(sys.argv)
	window = rcmanagerEditorWidget()
	window.show()
	sys.exit(app.exec_())


