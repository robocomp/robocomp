import importlib
from collections import ChainMap
from string import Template

from ..common.abstracttemplate import CustomTemplate, AbstractTemplate


class TemplatePython(AbstractTemplate):
    def __init__(self, component):
        self.files = {
                'regular': [
                    'CMakeLists.txt', 'DoxyFile', 'README-RCNODE.txt', 'README.md', 'etc/config', 'src/main.py',
                    'src/genericworker.py', 'src/specificworker.py', 'src/mainUI.ui'
                ],
                'avoid_overwrite': [
                    'src/specificworker.py', 'src/mainUI.ui', 'README.md', 'etc/config'
                ],
                'servant_files': ["SERVANT.PY"],
                'template_path': "/home/robolab/robocomp/tools/robocompdsl/templates/templatePython/files/"
        }
        super(TemplatePython, self).__init__(component)


    def README_md(self):
        return {'component_name': self.component.name}

    def CMakeLists_txt(self):
        if self.component.gui is not None:
            wrap_python_ui="WRAP_PYTHON_UI( mainUI )"
        else:
            wrap_python_ui = ""

        return {'wrap_python_ui': wrap_python_ui,
                'component_name': self.component.name}

    def src_mainUI_ui(self):
        return {'gui_type': self.component.gui.widget,
                'component_name': self.component.name}
