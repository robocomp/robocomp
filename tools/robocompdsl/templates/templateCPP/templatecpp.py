import datetime
import importlib
from collections import ChainMap
from string import Template

from ..common.abstracttemplate import CustomTemplate, AbstractTemplate
from ..templateCPP.functions import servant

class TemplateCpp(AbstractTemplate):
    def __init__(self, component):
        self.files = {
            'regular': [
                'CMakeLists.txt', 'DoxyFile', 'README-RCNODE.txt', 'README.md', 'etc/config', 'src/main.cpp',
                'src/CMakeLists.txt', 'src/CMakeListsSpecific.txt', 'src/commonbehaviorI.h', 'src/commonbehaviorI.cpp',
                'src/genericmonitor.h', 'src/genericmonitor.cpp', 'src/config.h', 'src/specificmonitor.h',
                'src/specificmonitor.cpp', 'src/genericworker.h', 'src/genericworker.cpp', 'src/specificworker.h',
                'src/specificworker.cpp', 'src/mainUI.ui'
            ],
            'avoid_overwrite': [
                'src/specificworker.h', 'src/specificworker.cpp', 'src/CMakeListsSpecific.txt',
                'src/mainUI.ui', 'src/specificmonitor.h', 'src/specificmonitor.cpp', 'README.md',
                'etc/config'
            ],
            'servant_files': ["SERVANT.H", "SERVANT.CPP"],
            'template_path': "templateCPP/files/"
        }
        super(TemplateCpp, self).__init__(component)

    # def template_to_file(self, template, output_file, interface_name=None):
    #         with open(template, 'r') as istream:
    #             content = istream.read()
    #             template_dict = self.get_template_dict(template,interface_name)
    #             template_object = CustomTemplate(content, trimlines=False)
    #             file_content = template_object.substitute(**template_dict)
    #             with open(output_file, 'w') as ostream:
    #                 ostream.write(file_content)
    #
    # def template_to_file_interface(self, interface_name, template, output_file):
    #         with open(template, 'r') as istream:
    #             content = istream.read()
    #             function_name = template.replace("templateCPP/functions/", "").replace('.', '_').replace('/', '.')
    #             if hasattr(self,function_name):
    #                 function = getattr(self, function_name)
    #                 template_dict = function(interface_name)
    #             # Dynamically import functions needed for this template file
    #             else:
    #                 functions_file = template.replace("templateCPP/functions/", "").replace('.', '_').replace('/','.')
    #                 functions = importlib.import_module("templateCPP.functions." + functions_file)
    #                 template_dict = functions.get_template_dict(self.component, interface_name)
    #             template_object = CustomTemplate(content)
    #             template_object.identifiers()
    #             file_content = template_object.substitute(**template_dict)
    #             with open(output_file, 'w') as ostream:
    #                 ostream.write(file_content)

    def SERVANT_H(self, interface_name):
        module = self.component.idsl_pool.moduleProviding(interface_name)
        return {
            'year': str(datetime.date.today().year),
            'interface_name': interface_name,
            'interface_name_upper': interface_name.upper(),
            'filename_without_extension': module['filename'].split('/')[-1].split('.')[0],
            'module_name': module['name'],
            'interface_methods_definition': servant.interface_methods_definition(self.component,
                                                                                 module,
                                                                                 interface_name)
        }

    def SERVANT_CPP(self, interface_name):
        return {
            'year': str(datetime.date.today().year),
            'interface_name': interface_name,
            'interface_name_lower': interface_name.lower(),
            'interface_methods_creation': servant.interface_methods_creation(self.component, interface_name)
        }

    def README_md(self):
        return {
            'component_name': self.component.name
        }

    def DoxyFile(self):
        return {
            'component_name': self.component.name
        }

    def CMakeLists_txt(self):
        return {
            'component_name': self.component.name
        }

    def src_mainUI_ui(self):
        return {'gui_type': self.component.gui.widget,
                'component_name': self.component.name}

    def src_config_h(self):
        need_gui = ""
        if self.component.gui is not None:
            need_gui = "#define USE_QTGUI\n\n"
        return {
            'component_name': self.component.name,
            'need_gui': need_gui
        }

    def src_commonbehaviorI_h(self):
        if self.component.language.lower() == 'cpp':
            const = "const"
            ampersand = "&"
        else:
            const = ""
            ampersand = ""
        return {
            'year': str(datetime.date.today().year),
            'const': const,
            'ampersand': ampersand
        }

    def src_commonbehaviorI_cpp(self):
        if self.component.language.lower() == 'cpp':
            const = "const"
            ampersand = "&"
        else:
            const = ""
            ampersand = ""
        return {
            'year': str(datetime.date.today().year),
            'const': const,
            'ampersand': ampersand
        }

    # def README_RCNODE_txt(self):
    #     return {}
    #
    # def etc_config(self):
    #     return {}
    #
    # def src_CMakeListsSpecific_txt(self):
    #     return {}
    #
    # def src_genericmonitor_h(self):
    #     return {}
    #
    # def src_genericmonitor_cpp(self):
    #     return {}
    #
    # def src_specificmonitor_h(self):
    #     return {}
    #
    # def src_specificmonitor_cpp(self):
    #     return {}

