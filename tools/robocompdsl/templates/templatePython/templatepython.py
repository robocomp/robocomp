import os

from ..common.abstracttemplatesmanager import ComponentTemplatesManager


class TemplatesManagerPython(ComponentTemplatesManager):
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
                'template_path': "templatePython/files/"
        }
        super(TemplatesManagerPython, self).__init__(component)


    def _pre_generation_check(self, template_file):
        if self.component.language.lower() == 'python' and template_file == 'CMakeLists.txt' and self.component.gui is None: return True
        return False

    def _output_file_rename(self, output_path, template_file):
        if self.component.language.lower() == 'python' and template_file == 'src/main.py':
            ofile = os.path.join(output_path, 'src', self.component.name + '.py')
        else:
            ofile = os.path.join(output_path, template_file)
        return ofile

    def _post_generation_action(self, template_file, ofile):
        if template_file == 'src/main.py' and self.component.language.lower() == 'python':
            os.chmod(ofile, os.stat(ofile).st_mode | 0o111)


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
