

import sys

#from PySide2.QtWidgets import QApplication, QMainWindow, QTextEdit, QFileDialog, QWidget
#from PySide2.QtCore import QFile, QRegExp, Qt
from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QApplication, QFileDialog, QMainWindow, QMenu, QMessageBox, QTextEdit, QWidget, QGridLayout
from ui_gui import Ui_MainWindow


class RoboCompDSLGui(QMainWindow):
    def __init__(self):
        super(RoboCompDSLGui, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)


if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = RoboCompDSLGui()
    gui.show()

    sys.exit(app.exec_())
