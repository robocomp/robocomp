import datetime
import sys
from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, get_name_number, IDSLPool
from templates.common.templatedict import TemplateDict


class src_main_py(TemplateDict):
    def __init__(self, component):
        super(src_main_py, self).__init__()
        self.component = component
        self['import_qtwidgets'] = self.import_qtwidgets()
        self['app_creation'] = self.app_creation()
        self['qt_application_quit'] = self.qt_application_quit()
        self['qt_app_exec'] = self.qt_app_exec()

    def import_qtwidgets(self):
        result = ""
        if self.component.gui is not None:
            result += 'from PySide2 import QtCore\nfrom PySide2 import QtWidgets\n'
        return result

    def app_creation(self):
        result = ""
        if self.component.gui is not None:
            result += 'app = QtWidgets.QApplication(sys.argv)\n'
        return result

    def qt_application_quit(self):
        result = ""
        if self.component.gui is not None:
            result += 'QtCore.QCoreApplication.quit()\n'
        else:
            result += 'exit()\n'
        return result

    def qt_app_exec(self):
        result = ""
        if self.component.gui is not None:
            result += 'app.exec_()\n'
        return result
