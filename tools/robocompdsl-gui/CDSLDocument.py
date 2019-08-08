from PySide2.QtCore import *


class CDSLLanguage:
    CPP = "CPP"
    PYTHON = "Python"


class CDSLGui:
    QWIDGET = "QWidget"
    QDIALOG = "QDialog"
    QMAINWINDOW = "QMainWindow"


class CDSLDocument(QObject):
    languageChange = Signal(str)
    innerModelViewerChange = Signal(bool)
    agmagentChange = Signal(bool)
    guiChange = Signal(bool)
    guiTypeChange = Signal(str)
    nameChange = Signal(str)

    def __init__(self):
        super(CDSLDocument, self).__init__()
        self.doc = []
        self._component_name = ""
        self._communications = {"implements": [], "requires": [], "subscribesTo": [], "publishes": []}
        self._imports = set()
        self._requires = []
        self._language = ""
        self._gui = False
        self._gui_type = CDSLGui.QWIDGET
        self._options = []
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
        for imp in sorted(self._imports):
            doc_str += self._t() + "import \"" + imp + "\";\n"
        return doc_str

    def _generate_generic_communication(self, com_type):
        doc_str = ""
        if com_type in self._communications:
            if len(self._communications[com_type]) > 0:
                doc_str = self._t() + "%s " % com_type
                for pos, element in enumerate(sorted(self._communications[com_type])):
                    doc_str += element
                    if pos < len(self._communications[com_type]) - 1:
                        doc_str += ", "
                    else:
                        doc_str += ";\n"
        else:
            print("CDSLDocument._generate_generic_communication: invalid communication type: %s" % com_type)
        return doc_str

    def generate_implements(self):
        return self._generate_generic_communication("implements")

    def generate_requires(self):
        return self._generate_generic_communication("requires")

    def generate_publish(self):
        return self._generate_generic_communication("publishes")

    def generate_subscribes(self):
        return self._generate_generic_communication("subscribesTo")

    def generate_communications(self):
        doc_str = self._t() + "communications\n"
        doc_str += self.open_sec() + "\n"
        doc_str += self.generate_implements()
        doc_str += self.generate_requires()
        doc_str += self.generate_publish()
        doc_str += self.generate_subscribes()
        doc_str += self.close_sec() + ";\n"
        return doc_str

    def generate_language(self):
        doc_str = self._t() + "language " + self._language + ";\n"
        return doc_str

    def generate_ui(self):
        doc_str = ""
        if self._gui:
            doc_str = self._t() + "gui Qt(" + self._gui_type + ");\n"
        return doc_str

    def generate_options(self):
        doc_str = ""
        if len(self._options) > 0:
            doc_str = self._t() + "options "
            for pos, element in enumerate(self._options):
                doc_str += element
                if pos < len(self._options) - 1:
                    doc_str += ", "
                else:
                    doc_str += ";\n"
        return doc_str

    def generate_component(self):
        doc_str = "\ncomponent " + self._component_name
        doc_str += "\n" + self.open_sec() + "\n"
        doc_str += self.generate_communications()
        doc_str += self.generate_language()
        doc_str += self.generate_ui()
        doc_str += self.generate_options()
        doc_str += self.close_sec() + ";"
        return doc_str

    def generate_doc(self):
        doc_str = ""
        doc_str += self.generate_imports()
        doc_str += self.generate_component()
        return doc_str

    def clear_imports(self):
        self._imports.clear()

    def add_import(self, import_file):
        self._imports.add(import_file)

    def remove_import(self, import_file):
        if import_file in self._imports:
            self._imports.remove(import_file)

    def add_requires(self, require_name):
        self._communications["requires"].append(require_name)

    def remove_requires(self, require_name):
        if require_name in self._communications["requires"]:
            self._communications["requires"].remove(require_name)

    def add_publishes(self, publish_name):
        self._communications["publishes"].append(publish_name)

    def remove_publishes(self, publish_name):
        if publish_name in self._communications["publishes"]:
            self._communications["publishes"].remove(require_name)

    def add_subscribesTo(self, subscribe_name):
        self._communications["subscribesTo"].append(subscribe_name)

    def remove_subscribesTo(self, subscribe_name):
        if subscribe_name in self._communications["subscribesTo"]:
            self._communications["subscribesTo"].remove(require_name)

    def add_implements(self, implement_name):
        self._communications["implements"].append(implement_name)

    def remove_implements(self, implement_name):
        if implement_name in self._communications["implements"]:
            self._communications["implements"].remove(require_name)

    def add_communication(self, com_type, com_name):
        if com_type in self._communications:
            self._communications[com_type].append(com_name)
        else:
            print("CDSLDocument.add_communication: invalid communication type: %s" % com_type)

    def remove_communication(self, com_type, com_name):
        if com_type in self._communications:
            if com_name in self._communications[com_type]:
                self._communications[com_type].remove(com_name)
        else:
            print("CDSLDocument.add_communication: invalid communication type: %s" % com_type)

    def clear_communication(self, com_type):
        if com_type in self._communications:
            self._communications[com_type] = []
        else:
            print("CDSLDocument.add_communication: invalid communication type: %s" % com_type)

    def set_name(self, name):
        self._component_name = name

    def set_gui(self, gui):
        self._gui = gui

    def set_gui_type(self, gui_type):
        self._gui_type = gui_type

    def set_agmagent(self, agmagent):
        self.add_option("agmagent")
        agm_import = ['AGMExecutive.idsl', 'AGMCommonBehavior.idsl', 'AGMWorldModel.idsl']
        if agmagent is True:
            self.add_option("agmagent")
            # imports
            for file in agm_import:
                self.add_import(file)
            # communications
            if 'AGMCommonBehavior' not in self._communications['implements']:
                self.add_communication('implements', 'AGMCommonBehavior')
            if 'AGMExecutive' not in self._communications['requires']:
                self.add_communication('requires', 'AGMExecutive')
            if 'AGMExecutiveTopic' not in self._communications['subscribesTo']:
                self.add_communication('subscribesTo', 'AGMExecutiveTopic')
        else:
            self.remove_option("agmagent")
            for file in agm_import:
                self.remove_import(file)
            self.remove_communication('implements', 'AGMCommonBehavior')
            self.remove_communication('requires', 'AGMExecutive')
            self.remove_communication('subscribesTo', 'AGMExecutiveTopic')

    def set_innerModel(self, innerModel):
        if innerModel:
            self.add_option("innermodelviewer")
        else:
            self.remove_option("innermodelviewer")

    def add_option(self, option):
        if option not in self._options:
            self._options.append(option)

    def remove_option(self, option):
        if option in self._options:
            self._options.remove(option)

    def clear(self):
        self.doc = []
        self._component_name = ""
        self._communications = {"implements": [], "requires": [], "subscribesTo": [], "publishes": []}
        self._imports = set()
        self._requires = []
        self._language = ""
        self._gui = False

    def analize_language(self, s, loc, toks):
        lang = toks.language[0]
        #        print("analyze language: ", lang)
        if self._language != lang:
            self.set_language(lang)
            self.languageChange.emit(self.get_language())

    def set_language(self, language):
        self._language = language

    def get_language(self):
        return self._language

    def analize_options(self, s, loc, toks):
        names = [item.lower() for sublist in toks for item in sublist]
        #TODO: review
        if 'agmagent' in names:
            self.set_agmagent(True)
            self.agmagentChange.emit(True)
        elif 'agmagent' in self._options:
            self.set_agmagent(False)
            self.agmagentChange.emit(False)
        if 'innermodelviewer' in names:
            self.add_option('innermodelviewer')
            self.innerModelViewerChange.emit(True)
        elif 'innermodelviewer' in self._options:
            self.remove_option('innermodelviewer')
            self.innerModelViewerChange.emit(False)

    def analize_gui(self, s, loc, toks):
        gui = False
        if toks.gui:
            gui_type = toks.gui[1]
            gui = True
            if self._gui_type != gui_type:
                self.set_gui_type(gui_type)
        self.set_gui(gui)
        self.guiChange.emit(self.get_gui())
        self.guiTypeChange.emit(self.get_gui_type())

    def get_gui(self):
        return self._gui

    def get_gui_type(self):
        return self._gui_type

    def get_name(self):
        return self._component_name

    def analize_compname(self, s, loc, toks):
        self.set_name(toks[0])
        self.nameChange.emit(self.get_name())

