
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QFile, QRegExp, Qt
from PyQt5.QtGui import QFont, QSyntaxHighlighter, QTextCharFormat

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 540)
        MainWindow.setMinimumSize(QtCore.QSize(800, 540))
        MainWindow.setMaximumSize(QtCore.QSize(800, 540))
        MainWindow.setBaseSize(QtCore.QSize(0, 0))
        MainWindow.setWindowTitle("Create new component")

        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.centralWidget.setObjectName("centralWidget")

        # LAYOUTS
        self.formLayoutWidget = QtWidgets.QWidget(self.centralWidget)
        self.formLayoutWidget.setGeometry(QtCore.QRect(9, 9, 781, 61))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.formLayout = QtWidgets.QFormLayout(self.formLayoutWidget)
        self.formLayout.setSizeConstraint(QtWidgets.QLayout.SetMaximumSize)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setObjectName("formLayout")

        self.gridLayoutWidget = QtWidgets.QWidget(self.centralWidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(9, 79, 781, 301))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")

        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.centralWidget)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(9, 391, 781, 121))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")

        # COMPONENT NAME
        self.nameLabel = QtWidgets.QLabel(self.formLayoutWidget)
        self.nameLabel.setObjectName("nameLabel")
        self.nameLabel.setText("Component Name:")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.nameLabel)

        self.nameTextEdit = QtWidgets.QPlainTextEdit(self.formLayoutWidget)
        self.nameTextEdit.setMaximumSize(QtCore.QSize(16777215, 25))
        self.nameTextEdit.setObjectName("nameTextEdit")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.nameTextEdit)

        # DIRECTORY
        self.directoryTextEdit = QtWidgets.QPlainTextEdit(self.formLayoutWidget)
        self.directoryTextEdit.setMaximumSize(QtCore.QSize(16777215, 25))
        self.directoryTextEdit.setObjectName("directoryTextEdit")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.directoryTextEdit)
        self.directoryButton = QtWidgets.QPushButton(self.formLayoutWidget)
        self.directoryButton.setObjectName("directoryButton")
        self.directoryButton.setText("Select Directory")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.directoryButton)

        # INTERFACES
        self.interfacesLabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.interfacesLabel.setMaximumSize(QtCore.QSize(16777215, 18))
        self.interfacesLabel.setStyleSheet("background-color: rgb(186, 189, 182);")
        self.interfacesLabel.setObjectName("interfacesLabel")
        self.interfacesLabel.setText("Interfaces")
        self.gridLayout.addWidget(self.interfacesLabel, 0, 0, 1, 1)

        self.communicationsComboBox = QtWidgets.QComboBox(self.gridLayoutWidget)
        self.communicationsComboBox.setObjectName("communicationsComboBox")
        self.communicationsComboBox.addItem("publishes")
        self.communicationsComboBox.addItem("implements")
        self.communicationsComboBox.addItem("subscribesTo")
        self.communicationsComboBox.addItem("requires")
        self.gridLayout.addWidget(self.communicationsComboBox, 1, 0, 1, 1)

        self.interfacesListWidget = QtWidgets.QListWidget(self.gridLayoutWidget)
        self.interfacesListWidget.setMaximumSize(QtCore.QSize(160, 16777215))
        self.interfacesListWidget.setObjectName("interfacesListWidget")
        self.gridLayout.addWidget(self.interfacesListWidget, 2, 0, 7, 1)

        self.addButton = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.addButton.setObjectName("addButton")
        self.addButton.setText("Add Interface")
        self.gridLayout.addWidget(self.addButton, 9, 0, 1, 1)

        # MAIN TEXT
        self.setupEditor()

        # LANGUAGE
        self.languageLabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.languageLabel.setMinimumSize(QtCore.QSize(160, 0))
        self.languageLabel.setMaximumSize(QtCore.QSize(16777215, 18))
        self.languageLabel.setStyleSheet("background-color: rgb(186, 189, 182);")
        self.languageLabel.setObjectName("languageLabel")
        self.languageLabel.setText("Language")
        self.gridLayout.addWidget(self.languageLabel, 0, 2, 1, 1)

        self.languageComboBox = QtWidgets.QComboBox(self.gridLayoutWidget)
        self.languageComboBox.setObjectName("languageComboBox")
        self.languageComboBox.addItem("Python")
        self.languageComboBox.addItem("Cpp")
        self.languageComboBox.addItem("Cpp11")
        self.gridLayout.addWidget(self.languageComboBox, 1, 2, 1, 1)

        # GUI
        self.guiLabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.guiLabel.setMaximumSize(QtCore.QSize(16777215, 18))
        self.guiLabel.setStyleSheet("background-color: rgb(186, 189, 182);")
        self.guiLabel.setObjectName("guiLabel")
        self.guiLabel.setText("GUI")
        self.gridLayout.addWidget(self.guiLabel, 2, 2, 1, 1)

        self.guiCheckBox = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.guiCheckBox.setObjectName("guiCheckBox")
        self.guiCheckBox.setText("Window")
        self.gridLayout.addWidget(self.guiCheckBox, 3, 2, 1, 1)

        self.guiComboBox = QtWidgets.QComboBox(self.gridLayoutWidget)
        self.guiComboBox.setEnabled(True)
        self.guiComboBox.setObjectName("guiComboBox")
        self.guiComboBox.addItem("MainWindow")
        self.guiComboBox.addItem("QDialog")
        self.guiComboBox.addItem("QMainWindow")
        self.gridLayout.addWidget(self.guiComboBox, 4, 2, 1, 1)

        # OPTIONAL PARAMETERS
        self.optionalLabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.optionalLabel.setMaximumSize(QtCore.QSize(16777215, 18))
        self.optionalLabel.setStyleSheet("background-color: rgb(186, 189, 182);")
        self.optionalLabel.setObjectName("optionalLabel")
        self.optionalLabel.setText("Optional Parameters")
        self.gridLayout.addWidget(self.optionalLabel, 5, 2, 1, 1)

        # AGMAGENT
        self.agmagentCheckBox = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.agmagentCheckBox.setObjectName("agmagentCheckBox")
        self.agmagentCheckBox.setText("Agmagent")
        self.gridLayout.addWidget(self.agmagentCheckBox, 6, 2, 1, 1)

        # INNERMODEL VIEWER
        self.innermodelCheckBox = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.innermodelCheckBox.setObjectName("innermodelCheckBox")
        self.innermodelCheckBox.setText("InnermodelViewer")
        self.gridLayout.addWidget(self.innermodelCheckBox, 7, 2, 1, 1)

        # MODULES
        self.moduleCheckBox = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.moduleCheckBox.setObjectName("moduleCheckBox")
        self.moduleCheckBox.setText("Module")
        self.gridLayout.addWidget(self.moduleCheckBox, 8, 2, 1, 1)

        self.modulesComboBox = QtWidgets.QComboBox(self.gridLayoutWidget)
        self.modulesComboBox.setEnabled(True)
        self.modulesComboBox.setObjectName("modulesComboBox")
        self.modulesComboBox.addItem("ROS")
        self.modulesComboBox.addItem("Opencv2")
        self.modulesComboBox.addItem("PCL")
        self.gridLayout.addWidget(self.modulesComboBox, 9, 2, 1, 1)

        # TERMINAL
        self.terminalLabel = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.terminalLabel.setMaximumSize(QtCore.QSize(16777215, 18))
        self.terminalLabel.setStyleSheet("background-color: rgb(186, 189, 182);")
        self.terminalLabel.setObjectName("terminalLabel")
        self.terminalLabel.setText("Terminal")
        self.gridLayout_2.addWidget(self.terminalLabel, 0, 0, 1, 1)

        self.terminalTextEdit = QtWidgets.QTextEdit(self.gridLayoutWidget_2)
        self.terminalTextEdit.setStyleSheet("background-color: rgb(4, 11, 50);")
        self.terminalTextEdit.setObjectName("terminalTextEdit")
        self.gridLayout_2.addWidget(self.terminalTextEdit, 1, 0, 3, 1)

        # ACTION BUTTONS
        self.resetButton = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.resetButton.setMinimumSize(QtCore.QSize(160, 0))
        self.resetButton.setObjectName("resetButton")
        self.resetButton.setText("Reset")
        self.gridLayout_2.addWidget(self.resetButton, 1, 1, 1, 1)

        self.createButton = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.createButton.setObjectName("createButton")
        self.createButton.setText("Create CDSL")
        self.gridLayout_2.addWidget(self.createButton, 2, 1, 1, 1)

        self.generateButton = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.generateButton.setObjectName("generateButton")
        self.generateButton.setText("Generate")
        self.gridLayout_2.addWidget(self.generateButton, 3, 1, 1, 1)

        MainWindow.setCentralWidget(self.centralWidget)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)

        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def setupEditor(self):
        font = QFont()
        font.setFamily('Courier')
        font.setFixedPitch(True)
        font.setPointSize(10)

        self.mainTextEdit = QtWidgets.QTextEdit(self.gridLayoutWidget)
        self.mainTextEdit.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.mainTextEdit.setObjectName("mainTextEdit")
        self.gridLayout.addWidget(self.mainTextEdit, 0, 1, 10, 1)

        self.highlighter = Highlighter(self.mainTextEdit.document())

class Highlighter(QSyntaxHighlighter):
    def __init__(self, parent=None):
        super(Highlighter, self).__init__(parent)

        keywordFormat = QTextCharFormat()
        keywordFormat.setForeground(Qt.darkBlue)
        keywordFormat.setFontWeight(QFont.Bold)

        keywordPatterns = ["\\bimport\\b", "\\bComponent\\b", "\\bCommunications\\b",
                "\\bpublishes\\b", "\\implements\\b", "\\subscribesTo\\b", "\\brequires\\b",
                "\\blanguage\\b"]

        self.highlightingRules = [(QRegExp(pattern), keywordFormat)
                for pattern in keywordPatterns]

        classFormat = QTextCharFormat()
        classFormat.setFontWeight(QFont.Bold)
        classFormat.setForeground(Qt.darkMagenta)
        self.highlightingRules.append((QRegExp("\\bQ[A-Za-z]+\\b"),
                classFormat))

        singleLineCommentFormat = QTextCharFormat()
        singleLineCommentFormat.setForeground(Qt.red)
        self.highlightingRules.append((QRegExp("//[^\n]*"),
                singleLineCommentFormat))

        self.multiLineCommentFormat = QTextCharFormat()
        self.multiLineCommentFormat.setForeground(Qt.red)

        quotationFormat = QTextCharFormat()
        quotationFormat.setForeground(Qt.darkGreen)
        self.highlightingRules.append((QRegExp("\".*\""), quotationFormat))

        functionFormat = QTextCharFormat()
        functionFormat.setFontItalic(True)
        functionFormat.setForeground(Qt.blue)
        self.highlightingRules.append((QRegExp("\\b[A-Za-z0-9_]+(?=\\()"),
                functionFormat))

        self.commentStartExpression = QRegExp("/\\*")
        self.commentEndExpression = QRegExp("\\*/")

    def highlightBlock(self, text):
        for pattern, format in self.highlightingRules:
            expression = QRegExp(pattern)
            index = expression.indexIn(text)
            while index >= 0:
                length = expression.matchedLength()
                self.setFormat(index, length, format)
                index = expression.indexIn(text, index + length)

        self.setCurrentBlockState(0)

        startIndex = 0
        if self.previousBlockState() != 1:
            startIndex = self.commentStartExpression.indexIn(text)

        while startIndex >= 0:
            endIndex = self.commentEndExpression.indexIn(text, startIndex)

            if endIndex == -1:
                self.setCurrentBlockState(1)
                commentLength = len(text) - startIndex
            else:
                commentLength = endIndex - startIndex + self.commentEndExpression.matchedLength()

            self.setFormat(startIndex, commentLength,
                    self.multiLineCommentFormat)
            startIndex = self.commentStartExpression.indexIn(text,
                    startIndex + commentLength);

