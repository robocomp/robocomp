import os

from ..common.abstracttemplatesmanager import ComponentTemplatesManager
from templates.common.plugin_collection import PluginCollection
from . import plugins


class TemplatesManagerPython(ComponentTemplatesManager):
    def __init__(self, component):
        self.files = {
                'regular': [
                    'CMakeLists.txt', 'DoxyFile', 'README-RCNODE.txt', 'README.md', 'etc/config', 'src/main.py',
                    'src/genericworker.py', 'src/specificworker.py', 'src/interfaces.py', 'src/mainUI.ui', 'src/CMakeLists.txt'
                ],
                'avoid_overwrite': [
                    'src/specificworker.py', 'src/mainUI.ui', 'README.md', 'etc/config'
                ],
                'servant_files': ["SERVANT.PY"],
                'template_path': "templatePython/files/"
        }
        current_plugins = PluginCollection(plugins.__name__)
        super(TemplatesManagerPython, self).__init__(component, current_plugins)


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




