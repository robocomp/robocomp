# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gui.ui'
#
# Created: Wed Nov 26 22:19:36 2014
#      by: PyQt4 UI code generator 4.11.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_ReplayMainWindow(object):
    def setupUi(self, ReplayMainWindow):
        ReplayMainWindow.setObjectName(_fromUtf8("ReplayMainWindow"))
        ReplayMainWindow.resize(1069, 918)
        self.centralwidget = QtGui.QWidget(ReplayMainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.mdiArea = QtGui.QMdiArea(self.centralwidget)
        self.mdiArea.setObjectName(_fromUtf8("mdiArea"))
        self.verticalLayout_3.addWidget(self.mdiArea)
        self.controls = QtGui.QHBoxLayout()
        self.controls.setObjectName(_fromUtf8("controls"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.PPButton = QtGui.QPushButton(self.centralwidget)
        self.PPButton.setObjectName(_fromUtf8("PPButton"))
        self.verticalLayout.addWidget(self.PPButton)
        self.forceButton = QtGui.QPushButton(self.centralwidget)
        self.forceButton.setEnabled(False)
        self.forceButton.setObjectName(_fromUtf8("forceButton"))
        self.verticalLayout.addWidget(self.forceButton)
        self.controls.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout_2.addWidget(self.label)
        self.spinBox = QtGui.QDoubleSpinBox(self.centralwidget)
        self.spinBox.setDecimals(3)
        self.spinBox.setMaximum(5.0)
        self.spinBox.setSingleStep(0.1)
        self.spinBox.setProperty("value", 1.0)
        self.spinBox.setObjectName(_fromUtf8("spinBox"))
        self.verticalLayout_2.addWidget(self.spinBox)
        self.controls.addLayout(self.verticalLayout_2)
        self.slider = QtGui.QSlider(self.centralwidget)
        self.slider.setOrientation(QtCore.Qt.Horizontal)
        self.slider.setObjectName(_fromUtf8("slider"))
        self.controls.addWidget(self.slider)
        self.verticalLayout_3.addLayout(self.controls)
        ReplayMainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(ReplayMainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1069, 27))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuOptions = QtGui.QMenu(self.menubar)
        self.menuOptions.setObjectName(_fromUtf8("menuOptions"))
        ReplayMainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(ReplayMainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        ReplayMainWindow.setStatusBar(self.statusbar)
        self.actionCascade = QtGui.QAction(ReplayMainWindow)
        self.actionCascade.setObjectName(_fromUtf8("actionCascade"))
        self.actionTile = QtGui.QAction(ReplayMainWindow)
        self.actionTile.setObjectName(_fromUtf8("actionTile"))
        self.menuOptions.addAction(self.actionCascade)
        self.menuOptions.addAction(self.actionTile)
        self.menubar.addAction(self.menuOptions.menuAction())

        self.retranslateUi(ReplayMainWindow)
        QtCore.QMetaObject.connectSlotsByName(ReplayMainWindow)

    def retranslateUi(self, ReplayMainWindow):
        ReplayMainWindow.setWindowTitle(_translate("ReplayMainWindow", "replayComp", None))
        self.PPButton.setText(_translate("ReplayMainWindow", "Pause", None))
        self.forceButton.setText(_translate("ReplayMainWindow", "Force frame", None))
        self.label.setText(_translate("ReplayMainWindow", "Speedup", None))
        self.menuOptions.setTitle(_translate("ReplayMainWindow", "View", None))
        self.actionCascade.setText(_translate("ReplayMainWindow", "Cascade windows", None))
        self.actionTile.setText(_translate("ReplayMainWindow", "Tile windows", None))

