import os
import re
import signal
import sys

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class BColors:
	HEADER = '\033[95m'
	OKBLUE = '\033[94m'
	OKGREEN = '\033[92m'
	WARNING = '\033[93m'
	FAIL = '\033[91m'
	ENDC = '\033[0m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'


# DETECT THE ROBOCOMP INSTALLATION TO IMPORT RCPORTCHECKER CLASS
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOCOMP = ''
ROBOCOMP_COMP_DIR = os.path.join(os.path.expanduser("~"), "robocomp", "components")
ROBOCOMPDSL_DIR = os.path.join(CURRENT_DIR, "..", "robocompdsl")
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	default_robocomp_path = '/opt/robocomp'
	print('ROBOCOMP environment variable not set! Trying default directory (%s)' % (default_robocomp_path))
	if os.path.exists(default_robocomp_path) and os.path.isdir(default_robocomp_path):
		if not os.listdir(default_robocomp_path):
			print("Default Robocomp directory (%s) exists but it's empty. Exiting!" % (default_robocomp_path))
			sys.exit()
		else:
			ROBOCOMP = default_robocomp_path
	else:
		print("Default Robocomp directory (%s) doesn't exists. Exiting!" % (default_robocomp_path))
		sys.exit()
# sys.path.append(os.path.join(ROBOCOMP, "tools/rcportchecker"))

ROBOCOMP_INTERFACES = os.path.join(ROBOCOMP, "interfaces")
if not os.path.isdir(ROBOCOMP_INTERFACES):
	new_path = os.path.join(os.path.expanduser("~"), "robocomp", "interfaces")
	print('ROBOCOMP INTERFACES not found at %s! Trying HOME directory (%s)' % (ROBOCOMP_INTERFACES, new_path))
	ROBOCOMP_INTERFACES = new_path
	if not os.path.isdir(ROBOCOMP_INTERFACES):
		print("Default Robocomp INTERFACES directory (%s) doesn't exists. Exiting!" % (ROBOCOMP_INTERFACES))
		sys.exit()


class CDSLLanguage:
	CPP = "CPP"
	PYTHON = "Python"


class CDSLDocument:
	def __init__(self):
		self.doc = []
		self._component_name = ""
		self._communications = {"implements": [], "requires": [], "subscribesTo": [], "publishes": []}
		self._imports = []
		self._requires = []
		self._language = CDSLLanguage.PYTHON
		self._gui = False
		self._current_indentation = 0

	def _t(self):
		doc_str = '\t' * self._current_indentation
		return doc_str

	def open_sec(self):
		doc_str = self._t() + "{"
		self._current_indentation += 1
		return doc_str

	def close_sec(self):
		self._current_indentation -= 1
		doc_str = self._t() + "}"
		return doc_str

	def generate_imports(self):
		doc_str = ""
		for imp in self._imports:
			doc_str += self._t() + "import \"" + imp + "\"\n"
		return doc_str

	def _generate_generic_comunication(self, com_type):
		doc_str = ""
		if com_type in self._communications:
			if len(self._communications[com_type]) > 0:
				doc_str = self._t() + "%s " % (com_type)
				for pos, element in enumerate(self._communications[com_type]):
					doc_str += element
					if pos < len(self._communications[com_type]) - 1:
						doc_str += ", "
					else:
						doc_str += ";\n"
		else:
			print ("CDSLDocument._generate_generic_comunication: invalid communication type: %s" % com_type)
		return doc_str

	def generate_implements(self):
		return self._generate_generic_comunication("implements")

	def generate_requires(self):
		return self._generate_generic_comunication("requires")

	def generate_publish(self):
		return self._generate_generic_comunication("publishes")

	def generate_subscribes(self):
		return self._generate_generic_comunication("subscribesTo")

	def generate_comunications(self):
		doc_str = self._t() + "Communications\n"
		doc_str += self.open_sec() + "\n"
		doc_str += self.generate_implements()
		doc_str += self.generate_requires()
		doc_str += self.generate_publish()
		doc_str += self.generate_subscribes()
		doc_str += self.close_sec() + ";\n"
		return doc_str

	def generate_language(self):
		doc_str = self._t() + "language " + self._language + ";\n";
		return doc_str

	def generate_ui(self):
		doc_str = ""
		if self._gui:
			doc_str = self._t() + "gui Qt(QWidget);\n"
		return doc_str

	def generate_component(self):
		doc_str = "\nComponent " + self._component_name
		doc_str += "\n" + self.open_sec() + "\n"
		doc_str += self.generate_comunications()
		doc_str += self.generate_language()
		doc_str += self.generate_ui()
		doc_str += self.close_sec() + ";"
		return doc_str

	def generate_doc(self):
		doc_str = ""
		doc_str += self.generate_imports()
		doc_str += self.generate_component()
		return doc_str

	def clear_imports(self):
		self._imports = []

	def add_import(self, import_file):
		self._imports.append(import_file)

	def add_require(self, require_name):
		self._communications["requires"].append(require_name)

	def add_publish(self, publish_name):
		self._communications["publishes"].append(publish_name)

	def add_subscribe(self, subscribe_name):
		self._communications["subscribesTo"].append(subscribe_name)

	def add_implement(self, implement_name):
		self._communications["implements"].append(implement_name)

	def add_comunication(self, com_type, com_name):
		if com_type in self._communications:
			self._communications[com_type].append(com_name)
		else:
			print ("CDSLDocument.add_comunication: invalid communication type: %s" % com_type)

	def clear_comunication(self, com_type):
		if com_type in self._communications:
			self._communications[com_type] = []
		else:
			print ("CDSLDocument.add_comunication: invalid communication type: %s" % com_type)

	def set_name(self, name):
		self._component_name = name

	def set_language(self, language):
		self._language = language

	def set_qui(self, gui):
		self._gui = gui


class RobocompDslGui(QMainWindow):
	def __init__(self, parent=None):
		super(RobocompDslGui, self).__init__(parent)
		self.setWindowTitle("Create new component")
		# self._idsl_paths = []
		self._communications = {"implements": [], "requires": [], "subscribesTo": [], "publishes": []}
		self._interfaces = {}
		self._cdsl_doc = CDSLDocument()
		self._command_process = QProcess()

		self._main_widget = QWidget()
		self._main_layout = QVBoxLayout()
		self.setCentralWidget(self._main_widget)

		self._name_layout = QHBoxLayout()
		self._name_line_edit = QLineEdit()
		self._name_line_edit.textEdited.connect(self.update_component_name)
		self._name_line_edit.setPlaceholderText("New component name")
		self._name_layout.addWidget(self._name_line_edit)
		self._name_layout.addStretch()

		# DIRECTORY SELECTION
		self._dir_line_edit = QLineEdit()
		# self._dir_line_edit.textEdited.connect(self.update_completer)
		self._dir_completer = QCompleter()
		self._dir_completer_model = QFileSystemModel()
		if os.path.isdir(ROBOCOMP_COMP_DIR):
			self._dir_line_edit.setText(ROBOCOMP_COMP_DIR)
			self._dir_completer_model.setRootPath(ROBOCOMP_COMP_DIR)
		self._dir_completer.setModel(self._dir_completer_model)
		self._dir_line_edit.setCompleter(self._dir_completer)

		self._dir_button = QPushButton("Select directory")
		self._dir_button.clicked.connect(self.set_output_directory)
		self._dir_layout = QHBoxLayout()
		self._dir_layout.addWidget(self._dir_line_edit)
		self._dir_layout.addWidget(self._dir_button)

		# LIST OF ROBOCOMP INTERFACES
		self._interface_list = QListWidget()
		self._interface_list.setSelectionMode(QAbstractItemView.ExtendedSelection)
		self._interface_list.itemSelectionChanged.connect(self.set_comunication)

		# LIST OF CONNECTION TyPES
		self._type_combo_box = QComboBox()
		self._type_combo_box.addItems(["publishes", "implements", "subscribesTo", "requires"])
		self._type_combo_box.currentIndexChanged.connect(self.reselect_existing)

		# BUTTON TO ADD A NEW CONNECTION
		# self._add_connection_button = QPushButton("Add")
		# self._add_connection_button.clicked.connect(self.add_new_comunication)
		self._add_connection_layout = QHBoxLayout()
		# self._add_connection_layout.addWidget(self._add_connection_button)
		self._language_combo_box = QComboBox()
		self._language_combo_box.addItems(["Python", "Cpp", "Cpp11"])
		self._language_combo_box.currentIndexChanged.connect(self.update_language)
		self._add_connection_layout.addWidget(self._language_combo_box)
		self._add_connection_layout.addStretch()
		self._gui_check_box = QCheckBox()
		self._gui_check_box.stateChanged.connect(self.update_gui_selection)
		self._gui_label = QLabel("Use Qt GUI")
		self._add_connection_layout.addWidget(self._gui_label)
		self._add_connection_layout.addWidget(self._gui_check_box)

		# WIDGET CONTAINING INTERFACES AND TYPES
		self._selection_layout = QVBoxLayout()
		self._selection_layout.addWidget(self._type_combo_box)
		self._selection_layout.addWidget(self._interface_list)
		self._selection_layout.addLayout(self._add_connection_layout)
		self._selection_widget = QWidget()
		self._selection_widget.setLayout(self._selection_layout)

		# TEXT EDITOR WITH THE RESULTING CDSL CODE
		self._editor = QTextEdit(self)
		self._editor.setHtml("")

		self._document = self._editor.document()
		self._component_directory = None

		# SPLITTER WITH THE SELECTION AND THE CODE
		self._body_splitter = QSplitter(Qt.Horizontal)
		self._body_splitter.addWidget(self._selection_widget)
		self._body_splitter.addWidget(self._editor)
		self._body_splitter.setStretchFactor(0, 2)
		self._body_splitter.setStretchFactor(1, 9)

		# CREATION BUTTONS
		self._create_button = QPushButton("Create .cdsl")
		self._create_button.clicked.connect(self.write_cdsl_file)
		self._creation_layout = QHBoxLayout()
		self._creation_layout.addStretch()
		self._creation_layout.addWidget(self._create_button)

		self._console = QConsole()
		self._command_process.readyReadStandardOutput.connect(self._console.standard_output)
		self._command_process.readyReadStandardError.connect(self._console.error_output)

		# ADDING WIDGETS TO MAIN LAYOUT
		self._main_widget.setLayout(self._main_layout)
		self._main_layout.addLayout(self._name_layout)
		self._main_layout.addLayout(self._dir_layout)
		self._main_layout.addWidget(self._body_splitter)
		self._main_layout.addLayout(self._creation_layout)
		self._main_layout.addWidget(self._console)
		self.setMinimumSize(800,500)
		self._editor.setText(self._cdsl_doc.generate_doc())

	# self.editor->show();

	# def update_completer(self, path):
	# 	print "update_completer %s"%path
	# 	info = QFileInfo(path)
	# 	if info.exists() and info.isDir():
	# 			if not path.endswith(os.path.pathsep):
	# 				new_path = os.path.join(path, os.sep)
	# 				# self._dir_line_edit.setText(new_path)
	# 			all_dirs_output = [dI for dI in os.listdir(path) if os.path.isdir(os.path.join(path, dI))]
	# 			print all_dirs_output
	# 			self._dir_completer.complete()

	def load_idsl_files(self, fullpath=None):
		if fullpath is None:
			fullpath = ROBOCOMP_INTERFACES
		idsls_dir = os.path.join(ROBOCOMP_INTERFACES, "IDSLs")
		if os.path.isdir(idsls_dir):
			for full_filename in os.listdir(idsls_dir):
				file_name, file_extension = os.path.splitext(full_filename)
				if "idsl" in file_extension.lower():
					full_idsl_path = os.path.join(idsls_dir, full_filename)
					# self._idsl_paths.append(os.path.join(idsls_dir,full_filename))
					self.parse_idsl_file(full_idsl_path)
		self._interface_list.addItems(list(self._interfaces.keys()))

	def parse_idsl_file(self, fullpath):

		with open(fullpath, 'r') as fin:
			interface_name = None
			for line in fin:
				result = re.findall(r'^\s*interface\s+(\w+)\s*\{?\s*$', line, flags=re.MULTILINE)
				if len(result) > 0:
					interface_name = result[0]
			print("%s for idsl %s" % (interface_name, fullpath))
			if interface_name is not None:
				self._interfaces[interface_name] = fullpath

	def add_new_comunication(self):
		interface_names = self._interface_list.selectedItems()
		com_type = str(self._type_combo_box.currentText())
		for iface_name_item in interface_names:
			iface_name = str(iface_name_item.text())
			self._communications[com_type].append(iface_name)
			idsl_full_path = self._interfaces[iface_name]
			idsl_full_filename = os.path.basename(idsl_full_path)
			self._cdsl_doc.add_comunication(com_type, iface_name)
			self._cdsl_doc.add_import(idsl_full_filename)
		self.update_editor()

	def set_comunication(self):
		interface_names = self._interface_list.selectedItems()
		com_type = str(self._type_combo_box.currentText())
		self._communications[com_type] = []
		self._cdsl_doc.clear_comunication(com_type)
		for iface_name_item in interface_names:
			iface_name = str(iface_name_item.text())
			self._communications[com_type].append(iface_name)
			self._cdsl_doc.add_comunication(com_type, iface_name)
		self.update_imports()
		self.update_editor()

	def update_imports(self):
		self._cdsl_doc.clear_imports()
		for com_type in self._communications:
			for iface_name in self._communications[com_type]:
				idsl_full_path = self._interfaces[iface_name]
				idsl_full_filename = os.path.basename(idsl_full_path)
				self._cdsl_doc.add_import(idsl_full_filename)

	def update_language(self):
		language = self._language_combo_box.currentText()
		self._cdsl_doc.set_language(str(language))
		self.update_editor()

	def update_gui_selection(self):
		checked = self._gui_check_box.isChecked()
		if checked:
			self._cdsl_doc.set_qui(True)
		else:
			self._cdsl_doc.set_qui(False)
		self.update_editor()

	def update_component_name(self, name):
		self._cdsl_doc.set_name(name)
		self.update_editor()

	def update_editor(self):
		self._editor.setText(self._cdsl_doc.generate_doc())

	def set_output_directory(self):
		dir_set = False
		while not dir_set:
			dir = QFileDialog.getExistingDirectory(self, "Select Directory",
												   ROBOCOMP_COMP_DIR,
												   QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
			if self.check_dir_is_empty(str(dir)):
				self._dir_line_edit.setText(dir)
				dir_set = True

	def write_cdsl_file(self):
		component_dir = str(self._dir_line_edit.text())
		text = self._cdsl_doc.generate_doc()
		if not self._name_line_edit.text():
			component_name, ok = QInputDialog.getText(self, 'No component name set', 'Enter component name:')
			if ok:
				self.update_component_name(component_name)
				self._name_line_edit.setText(component_name)
			else:
				return False

		if not os.path.exists(component_dir):
			if QMessageBox.Yes == QMessageBox.question(self,
													   "Directory doesn't exist.",
													   "Do you want create the directory %s?" % component_dir,
													   QMessageBox.Yes | QMessageBox.No):
				os.makedirs(component_dir)
			else:
				QMessageBox.question(self,
									 "Directory not exist",
									 "Can't create a component witout a valid directory")
				return False

		file_path = os.path.join(component_dir, str(self._name_line_edit.text()) + ".cdsl")
		if os.path.exists(file_path):
			if QMessageBox.No == QMessageBox.question(self,
													  "File already exists",
													  "Do you want to overwrite?",
													  QMessageBox.Yes | QMessageBox.No):
				return False

		with open(file_path, 'w') as the_file:
			the_file.write(text)
		self.execute_robocomp_cdsl()
		return True

	def execute_robocomp_cdsl(self):
		cdsl_file_path = os.path.join(str(self._dir_line_edit.text()), str(self._name_line_edit.text()) + ".cdsl")
		command = "python -u %s/robocompdsl.py %s %s" % (
		ROBOCOMPDSL_DIR, cdsl_file_path, os.path.join(str(self._dir_line_edit.text())))
		self._console.append_custom_text("%s\n" % command)
		self._command_process.start(command, QProcess.Unbuffered | QProcess.ReadWrite)

	def reselect_existing(self):
		com_type = self._type_combo_box.currentText()
		selected = self._communications[com_type]
		self._interface_list.clearSelection()
		for iface in selected:
			items = self._interface_list.findItems(iface, Qt.MatchFlag.MatchExactly)
			if len(items) > 0:
				item = items[0]
				item.setSelected(True)

	def check_dir_is_empty(self, dir_path):
		if len(os.listdir(dir_path)) > 0:
			msgBox = QMessageBox()
			msgBox.setWindowTitle("Directory not empty")
			msgBox.setText("The selected directory is not empty.\n"
						   "For a new Component you usually want a new directory.\n"
						   "Do you want to use this directory anyway?")
			msgBox.setStandardButtons(QMessageBox.Yes)
			msgBox.addButton(QMessageBox.No)
			msgBox.setDefaultButton(QMessageBox.No)
			if msgBox.exec_() == QMessageBox.Yes:
				return True
			else:
				return False
		else:
			return True


class QConsole(QTextEdit):
	def __init__(self, parent=None):
		super(QConsole, self).__init__(parent)
		font = QFont("Monospace",9)
		font.setStyleHint(QFont.TypeWriter)
		self.setFont(font)
		# self.setFontWeight(QFont.Light)
		# self.setFontPointSize(9)
		self.setTextColor(QColor("LightGreen"))
		p = self.palette()
		p.setColor(QPalette.Base, QColor(0, 0, 0))
		self.setPalette(p)
		self.setText(">\n")

	def append_custom_text(self, text):
		self.setTextColor(QColor("white"))
		self.append(text)

	def standard_output(self):
		self.setTextColor(QColor("LightGreen"))
		process = self.sender()
		text = process.readAllStandardOutput()
		self.append(str(text))

	def error_output(self):
		self.setTextColor(QColor("red"))
		process = self.sender()
		text = process.readAllStandardError()
		self.append(str(text))


if __name__ == '__main__':
	app = QApplication(sys.argv)
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	main_window = RobocompDslGui()
	main_window.show()
	main_window.load_idsl_files()

	doc = CDSLDocument()
	doc._imports.append("HandDetection.idsl")
	doc._imports.append("CameraSimple.idsl")
	doc._imports.append("RGBD.idsl")
	doc._imports.append("CommonBehavior.idsl")
	doc._imports.append("TvGames.idsl")
	doc._imports.append("GetAprilTags.idsl")
	doc._imports.append("TouchPoints.idsl")
	doc._component_name = "tvgames"
	doc._communications["requires"] = ["HandDetection", "CameraSimple", " RGBD", "GetAprilTags"]
	doc._communications["implements"] = ["CommonBehavior", "TvGames"]
	doc._communications["publishes"] = ["TouchPoints"]
	doc._communications["subscribesTo"] = ["MisCo"]
	doc._language = CDSLLanguage.PYTHON
	doc._gui = True
	print (doc.generate_doc())

	app.exec_()
