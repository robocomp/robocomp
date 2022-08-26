# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'formManager.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(805, 600)
        MainWindow.setMinimumSize(QSize(0, 10))
        MainWindow.setBaseSize(QSize(0, 10))
        self.actionOpen = QAction(MainWindow)
        self.actionOpen.setObjectName(u"actionOpen")
        icon = QIcon()
        icon.addFile(u"share/rcmanager/1465393728_open-file.png", QSize(), QIcon.Normal, QIcon.Off)
        self.actionOpen.setIcon(icon)
        self.actionSave = QAction(MainWindow)
        self.actionSave.setObjectName(u"actionSave")
        icon1 = QIcon()
        icon1.addFile(u"share/rcmanager/1465394415_floppy.png", QSize(), QIcon.Normal, QIcon.Off)
        self.actionSave.setIcon(icon1)
        self.actionEdit = QAction(MainWindow)
        self.actionEdit.setObjectName(u"actionEdit")
        self.actionUndock = QAction(MainWindow)
        self.actionUndock.setObjectName(u"actionUndock")
        self.actionExit = QAction(MainWindow)
        self.actionExit.setObjectName(u"actionExit")
        icon2 = QIcon()
        icon2.addFile(u"share/rcmanager/quit_icon.png", QSize(), QIcon.Normal, QIcon.Off)
        self.actionExit.setIcon(icon2)
        self.actionSetting = QAction(MainWindow)
        self.actionSetting.setObjectName(u"actionSetting")
        icon3 = QIcon()
        icon3.addFile(u"share/rcmanager/1465394550_Settings.png", QSize(), QIcon.Normal, QIcon.Off)
        self.actionSetting.setIcon(icon3)
        self.actionSetting_2 = QAction(MainWindow)
        self.actionSetting_2.setObjectName(u"actionSetting_2")
        self.actionON = QAction(MainWindow)
        self.actionON.setObjectName(u"actionON")
        icon4 = QIcon()
        icon4.addFile(u"share/rcmanager/simulionOn.png", QSize(), QIcon.Normal, QIcon.Off)
        self.actionON.setIcon(icon4)
        self.actionOFF = QAction(MainWindow)
        self.actionOFF.setObjectName(u"actionOFF")
        icon5 = QIcon()
        icon5.addFile(u"share/rcmanager/simulatorOFF.png", QSize(), QIcon.Normal, QIcon.Off)
        self.actionOFF.setIcon(icon5)
        self.actionSetting_3 = QAction(MainWindow)
        self.actionSetting_3.setObjectName(u"actionSetting_3")
        self.actionSetting_4 = QAction(MainWindow)
        self.actionSetting_4.setObjectName(u"actionSetting_4")
        self.actionEditor_2 = QAction(MainWindow)
        self.actionEditor_2.setObjectName(u"actionEditor_2")
        self.actionControlPanel = QAction(MainWindow)
        self.actionControlPanel.setObjectName(u"actionControlPanel")
        self.actionSet_Log_File = QAction(MainWindow)
        self.actionSet_Log_File.setObjectName(u"actionSet_Log_File")
        self.actionLogger = QAction(MainWindow)
        self.actionLogger.setObjectName(u"actionLogger")
        self.actionLogger.setCheckable(True)
        self.actionLogger.setChecked(True)
        self.actionComponent_List = QAction(MainWindow)
        self.actionComponent_List.setObjectName(u"actionComponent_List")
        self.actionComponent_List.setCheckable(True)
        self.actionComponent_List.setChecked(True)
        self.actionFull_Screen = QAction(MainWindow)
        self.actionFull_Screen.setObjectName(u"actionFull_Screen")
        self.actionFull_Screen.setCheckable(True)
        self.actionSet_Color = QAction(MainWindow)
        self.actionSet_Color.setObjectName(u"actionSet_Color")
        self.actionSave_As = QAction(MainWindow)
        self.actionSave_As.setObjectName(u"actionSave_As")
        self.actionSave_As.setIcon(icon1)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayout = QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setMaximumSize(QSize(16777215, 16777215))
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.gridLayout_4 = QGridLayout(self.tab)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalSpacer_5 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_5.addItem(self.horizontalSpacer_5)

        self.label_4 = QLabel(self.tab)
        self.label_4.setObjectName(u"label_4")

        self.horizontalLayout_5.addWidget(self.label_4)

        self.horizontalSpacer_6 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_5.addItem(self.horizontalSpacer_6)


        self.verticalLayout_3.addLayout(self.horizontalLayout_5)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_2 = QLabel(self.tab)
        self.label_2.setObjectName(u"label_2")

        self.horizontalLayout_3.addWidget(self.label_2)

        self.doubleSpinBox = QDoubleSpinBox(self.tab)
        self.doubleSpinBox.setObjectName(u"doubleSpinBox")

        self.horizontalLayout_3.addWidget(self.doubleSpinBox)

        self.pushButton_2 = QPushButton(self.tab)
        self.pushButton_2.setObjectName(u"pushButton_2")

        self.horizontalLayout_3.addWidget(self.pushButton_2)

        self.pushButton = QPushButton(self.tab)
        self.pushButton.setObjectName(u"pushButton")

        self.horizontalLayout_3.addWidget(self.pushButton)

        self.horizontalSpacer_3 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_3.addItem(self.horizontalSpacer_3)


        self.verticalLayout_3.addLayout(self.horizontalLayout_3)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_3 = QLabel(self.tab)
        self.label_3.setObjectName(u"label_3")

        self.horizontalLayout_4.addWidget(self.label_3)

        self.doubleSpinBox_2 = QDoubleSpinBox(self.tab)
        self.doubleSpinBox_2.setObjectName(u"doubleSpinBox_2")

        self.horizontalLayout_4.addWidget(self.doubleSpinBox_2)

        self.pushButton_3 = QPushButton(self.tab)
        self.pushButton_3.setObjectName(u"pushButton_3")

        self.horizontalLayout_4.addWidget(self.pushButton_3)

        self.pushButton_4 = QPushButton(self.tab)
        self.pushButton_4.setObjectName(u"pushButton_4")

        self.horizontalLayout_4.addWidget(self.pushButton_4)

        self.horizontalSpacer_4 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_4)


        self.verticalLayout_3.addLayout(self.horizontalLayout_4)

        self.textBrowser_2 = QTextBrowser(self.tab)
        self.textBrowser_2.setObjectName(u"textBrowser_2")

        self.verticalLayout_3.addWidget(self.textBrowser_2)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.horizontalSpacer_7 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_6.addItem(self.horizontalSpacer_7)

        self.pushButton_6 = QPushButton(self.tab)
        self.pushButton_6.setObjectName(u"pushButton_6")

        self.horizontalLayout_6.addWidget(self.pushButton_6)


        self.verticalLayout_3.addLayout(self.horizontalLayout_6)

        self.verticalSpacer_3 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer_3)


        self.gridLayout_4.addLayout(self.verticalLayout_3, 0, 0, 1, 1)

        self.tabWidget.addTab(self.tab, "")
        self.tab_3 = QWidget()
        self.tab_3.setObjectName(u"tab_3")
        self.gridLayout_5 = QGridLayout(self.tab_3)
        self.gridLayout_5.setObjectName(u"gridLayout_5")
        self.frame = QFrame(self.tab_3)
        self.frame.setObjectName(u"frame")
        self.frame.setFrameShape(QFrame.StyledPanel)
        self.frame.setFrameShadow(QFrame.Raised)
        self.gridLayout_8 = QGridLayout(self.frame)
        self.gridLayout_8.setObjectName(u"gridLayout_8")

        self.gridLayout_5.addWidget(self.frame, 1, 0, 1, 1)

        self.tabWidget.addTab(self.tab_3, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.verticalLayout_2 = QVBoxLayout(self.tab_2)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.widget_2 = QWidget(self.tab_2)
        self.widget_2.setObjectName(u"widget_2")
        self.horizontalLayout_2 = QHBoxLayout(self.widget_2)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")

        self.verticalLayout_2.addWidget(self.widget_2)

        self.tabWidget.addTab(self.tab_2, "")

        self.gridLayout.addWidget(self.tabWidget, 0, 0, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 805, 25))
        self.menuFile = QMenu(self.menubar)
        self.menuFile.setObjectName(u"menuFile")
        self.menuSimulations = QMenu(self.menubar)
        self.menuSimulations.setObjectName(u"menuSimulations")
        self.menuSimulator = QMenu(self.menuSimulations)
        self.menuSimulator.setObjectName(u"menuSimulator")
        self.menuControl_Panel = QMenu(self.menuSimulations)
        self.menuControl_Panel.setObjectName(u"menuControl_Panel")
        self.menuEditor = QMenu(self.menuSimulations)
        self.menuEditor.setObjectName(u"menuEditor")
        self.menuEdit = QMenu(self.menubar)
        self.menuEdit.setObjectName(u"menuEdit")
        self.menuHelp = QMenu(self.menubar)
        self.menuHelp.setObjectName(u"menuHelp")
        self.menuView_2 = QMenu(self.menubar)
        self.menuView_2.setObjectName(u"menuView_2")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.toolBar = QToolBar(MainWindow)
        self.toolBar.setObjectName(u"toolBar")
        self.toolBar.setMinimumSize(QSize(0, 10))
        self.toolBar.setBaseSize(QSize(0, 10))
        MainWindow.addToolBar(Qt.TopToolBarArea, self.toolBar)
        self.dockWidget = QDockWidget(MainWindow)
        self.dockWidget.setObjectName(u"dockWidget")
        sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.dockWidget.sizePolicy().hasHeightForWidth())
        self.dockWidget.setSizePolicy(sizePolicy)
        self.dockWidget.setMinimumSize(QSize(121, 144))
        self.dockWidget.setFeatures(QDockWidget.DockWidgetFloatable|QDockWidget.DockWidgetMovable|QDockWidget.DockWidgetVerticalTitleBar)
        self.dockWidgetContents = QWidget()
        self.dockWidgetContents.setObjectName(u"dockWidgetContents")
        self.gridLayout_3 = QGridLayout(self.dockWidgetContents)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.textBrowser = QTextBrowser(self.dockWidgetContents)
        self.textBrowser.setObjectName(u"textBrowser")
        font = QFont()
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.textBrowser.setFont(font)

        self.gridLayout_3.addWidget(self.textBrowser, 2, 0, 1, 1)

        self.label = QLabel(self.dockWidgetContents)
        self.label.setObjectName(u"label")

        self.gridLayout_3.addWidget(self.label, 0, 0, 1, 1)

        self.dockWidget.setWidget(self.dockWidgetContents)
        MainWindow.addDockWidget(Qt.BottomDockWidgetArea, self.dockWidget)
        self.dockWidget_2 = QDockWidget(MainWindow)
        self.dockWidget_2.setObjectName(u"dockWidget_2")
        self.dockWidgetContents_2 = QWidget()
        self.dockWidgetContents_2.setObjectName(u"dockWidgetContents_2")
        self.gridLayout_2 = QGridLayout(self.dockWidgetContents_2)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.widget = QWidget(self.dockWidgetContents_2)
        self.widget.setObjectName(u"widget")
        self.gridLayout_6 = QGridLayout(self.widget)
        self.gridLayout_6.setObjectName(u"gridLayout_6")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.lineEdit = QLineEdit(self.widget)
        self.lineEdit.setObjectName(u"lineEdit")

        self.horizontalLayout.addWidget(self.lineEdit)

        self.toolButton_2 = QToolButton(self.widget)
        self.toolButton_2.setObjectName(u"toolButton_2")
        icon6 = QIcon()
        icon6.addFile(u"../../../../../opt/robocomp/share/rcmanager/1465608687_search.png", QSize(), QIcon.Normal, QIcon.Off)
        self.toolButton_2.setIcon(icon6)

        self.horizontalLayout.addWidget(self.toolButton_2)

        self.toolButton = QToolButton(self.widget)
        self.toolButton.setObjectName(u"toolButton")
        icon7 = QIcon()
        icon7.addFile(u"../../../../../opt/robocomp/share/rcmanager/1465594390_sign-add.png", QSize(), QIcon.Normal, QIcon.Off)
        self.toolButton.setIcon(icon7)

        self.horizontalLayout.addWidget(self.toolButton)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)


        self.gridLayout_6.addLayout(self.horizontalLayout, 0, 0, 1, 1)


        self.gridLayout_2.addWidget(self.widget, 0, 0, 1, 1)

        self.scrollArea = QScrollArea(self.dockWidgetContents_2)
        self.scrollArea.setObjectName(u"scrollArea")
        self.scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scrollArea.setWidgetResizable(True)
        self.scrollAreaWidgetContents = QWidget()
        self.scrollAreaWidgetContents.setObjectName(u"scrollAreaWidgetContents")
        self.scrollAreaWidgetContents.setGeometry(QRect(0, 0, 236, 176))
        self.verticalLayout = QVBoxLayout(self.scrollAreaWidgetContents)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer)

        self.scrollArea.setWidget(self.scrollAreaWidgetContents)

        self.gridLayout_2.addWidget(self.scrollArea, 1, 0, 1, 1)

        self.dockWidget_2.setWidget(self.dockWidgetContents_2)
        MainWindow.addDockWidget(Qt.RightDockWidgetArea, self.dockWidget_2)

        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuEdit.menuAction())
        self.menubar.addAction(self.menuView_2.menuAction())
        self.menubar.addAction(self.menuSimulations.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())
        self.menuFile.addAction(self.actionOpen)
        self.menuFile.addAction(self.actionSave)
        self.menuFile.addAction(self.actionSave_As)
        self.menuFile.addAction(self.actionExit)
        self.menuSimulations.addAction(self.menuSimulator.menuAction())
        self.menuSimulations.addAction(self.menuControl_Panel.menuAction())
        self.menuSimulations.addAction(self.menuEditor.menuAction())
        self.menuSimulations.addAction(self.actionSet_Log_File)
        self.menuSimulations.addAction(self.actionSet_Color)
        self.menuSimulator.addAction(self.actionSetting_2)
        self.menuSimulator.addAction(self.actionON)
        self.menuSimulator.addAction(self.actionOFF)
        self.menuControl_Panel.addAction(self.actionSetting_3)
        self.menuEditor.addAction(self.actionSetting_4)
        self.menuEdit.addAction(self.actionSetting)
        self.menuView_2.addAction(self.actionEditor_2)
        self.menuView_2.addAction(self.actionControlPanel)
        self.menuView_2.addAction(self.actionLogger)
        self.menuView_2.addAction(self.actionComponent_List)
        self.menuView_2.addAction(self.actionFull_Screen)
        self.toolBar.addAction(self.actionSave)
        self.toolBar.addAction(self.actionOpen)
        self.toolBar.addSeparator()
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionOFF)
        self.toolBar.addAction(self.actionON)
        self.toolBar.addSeparator()

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(1)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.actionOpen.setText(QCoreApplication.translate("MainWindow", u"Open", None))
#if QT_CONFIG(shortcut)
        self.actionOpen.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+O", None))
#endif // QT_CONFIG(shortcut)
        self.actionSave.setText(QCoreApplication.translate("MainWindow", u"Save", None))
#if QT_CONFIG(shortcut)
        self.actionSave.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+S", None))
#endif // QT_CONFIG(shortcut)
        self.actionEdit.setText(QCoreApplication.translate("MainWindow", u"Edit", None))
        self.actionUndock.setText(QCoreApplication.translate("MainWindow", u"Undock", None))
        self.actionExit.setText(QCoreApplication.translate("MainWindow", u"Exit", None))
#if QT_CONFIG(shortcut)
        self.actionExit.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+Q", None))
#endif // QT_CONFIG(shortcut)
        self.actionSetting.setText(QCoreApplication.translate("MainWindow", u"Setting", None))
        self.actionSetting_2.setText(QCoreApplication.translate("MainWindow", u"Setting", None))
        self.actionON.setText(QCoreApplication.translate("MainWindow", u"ON", None))
        self.actionOFF.setText(QCoreApplication.translate("MainWindow", u"OFF", None))
        self.actionSetting_3.setText(QCoreApplication.translate("MainWindow", u"Setting", None))
        self.actionSetting_4.setText(QCoreApplication.translate("MainWindow", u"Setting", None))
        self.actionEditor_2.setText(QCoreApplication.translate("MainWindow", u"Editor", None))
        self.actionControlPanel.setText(QCoreApplication.translate("MainWindow", u"Control Panel", None))
        self.actionSet_Log_File.setText(QCoreApplication.translate("MainWindow", u"Set Log File", None))
        self.actionLogger.setText(QCoreApplication.translate("MainWindow", u"Logger", None))
        self.actionComponent_List.setText(QCoreApplication.translate("MainWindow", u"Component List", None))
        self.actionFull_Screen.setText(QCoreApplication.translate("MainWindow", u"Full Screen", None))
#if QT_CONFIG(shortcut)
        self.actionFull_Screen.setShortcut(QCoreApplication.translate("MainWindow", u"F11", None))
#endif // QT_CONFIG(shortcut)
        self.actionSet_Color.setText(QCoreApplication.translate("MainWindow", u"Set Color", None))
        self.actionSave_As.setText(QCoreApplication.translate("MainWindow", u"Save As...", None))
#if QT_CONFIG(shortcut)
        self.actionSave_As.setShortcut(QCoreApplication.translate("MainWindow", u"Ctrl+Shift+S", None))
#endif // QT_CONFIG(shortcut)
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Component Name: Time Ellapsed", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Frequency  ", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"Reset", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"Default", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Blah              ", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"Reset", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"Default", None))
        self.pushButton_6.setText(QCoreApplication.translate("MainWindow", u"Kill Youself", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"Control panel", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), QCoreApplication.translate("MainWindow", u"Graph", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"Text", None))
        self.menuFile.setTitle(QCoreApplication.translate("MainWindow", u"File", None))
        self.menuSimulations.setTitle(QCoreApplication.translate("MainWindow", u"Tools", None))
        self.menuSimulator.setTitle(QCoreApplication.translate("MainWindow", u"Simulator", None))
        self.menuControl_Panel.setTitle(QCoreApplication.translate("MainWindow", u"Control Panel", None))
        self.menuEditor.setTitle(QCoreApplication.translate("MainWindow", u"Editor", None))
        self.menuEdit.setTitle(QCoreApplication.translate("MainWindow", u"Edit", None))
        self.menuHelp.setTitle(QCoreApplication.translate("MainWindow", u"Help", None))
        self.menuView_2.setTitle(QCoreApplication.translate("MainWindow", u"View", None))
        self.toolBar.setWindowTitle(QCoreApplication.translate("MainWindow", u"toolBar", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Log", None))
        self.lineEdit.setText("")
#if QT_CONFIG(tooltip)
        self.toolButton_2.setToolTip(QCoreApplication.translate("MainWindow", u"Search Component", None))
#endif // QT_CONFIG(tooltip)
        self.toolButton_2.setText(QCoreApplication.translate("MainWindow", u"...", None))
#if QT_CONFIG(tooltip)
        self.toolButton.setToolTip(QCoreApplication.translate("MainWindow", u"Add new component", None))
#endif // QT_CONFIG(tooltip)
        self.toolButton.setText(QCoreApplication.translate("MainWindow", u"...", None))
    # retranslateUi

