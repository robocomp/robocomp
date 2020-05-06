import importlib
from collections import ChainMap
import datetime
from string import Template
from templatePython.functions import SERVANT_PY as servant
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import IDSLPool


class MyTemplate(Template):
    delimiter = '$'
    pattern = r'''
    (?P<previous>[^$\n]*)\$(?:
      (?P<escaped>\$) |   # Escape sequence of two delimiters
      (?P<named>[_a-z][_a-z0-9]*)      |   # delimiter and a Python identifier
      {(?P<braced>[_a-z][_a-z0-9]*)}   |   # delimiter and a braced identifier
      (?P<invalid>)              # Other ill-formed delimiter exprs
    )
    '''

    def __init__(self, template, trimlines=True):
        super(MyTemplate, self).__init__(template)
        self.trimlines = trimlines


    def substitute(*args, **kws):
        if not args:
            raise TypeError("descriptor 'substitute' of 'Template' object "
                            "needs an argument")
        self, *args = args  # allow the "self" keyword be passed
        if len(args) > 1:
            raise TypeError('Too many positional arguments')
        if not args:
            mapping = kws
        elif kws:
            mapping = ChainMap(kws, args[0])
        else:
            mapping = args[0]

        def reindent(previous, string):
            if previous.strip() == '':
                out_lines = []
                lines = string.splitlines()
                if len(lines)>0:
                    if self.trimlines:
                        if lines and lines[0].strip() == '':
                            del lines[0]
                        if lines and lines[-1].strip() == '':
                            del lines[-1]
                    for line in lines:
                        if line.strip() != '':
                            out_lines.append(previous + line)
                        else:
                            out_lines.append(line)
                return '\n'.join(out_lines)
            else:
                return previous+string

        # Helper function for .sub()
        def convert(mo):
            # Check the most common path first.
            named = mo.group('named') or mo.group('braced')
            if named is not None:
                converted = reindent(mo.group('previous'), str(mapping[named]))
                if converted != '':
                    return converted
                else:
                    return "<LINEREMOVE>"
            if mo.group('escaped') is not None:
                return mo.group('previous')+self.delimiter
            if mo.group('invalid') is not None:
                self._invalid(mo)
            raise ValueError('Unrecognized named group in pattern',
                             self.pattern)
        substituted = self.pattern.sub(convert, self.template)
        # The only way to remove extra lines that template leaves.
        return substituted.replace('<LINEREMOVE>\n','')

    def identifiers(self):
        identifiers = []
        results = self.pattern.findall(self.template)
        for result in results:
            if result[3] != '' and result[3] not in identifiers:
                identifiers.append(result[3])
        return identifiers



class AbstractTemplate:
    def __init__(self, component):
        self.component = component

    def template_to_file(self, template, output_file):
            with open(template, 'r') as istream:
                content = istream.read()
                function_name = template.split('/')[-1].replace('.', '_').replace('-','_')
                if hasattr(self,function_name):
                    function = getattr(self, function_name)
                    template_dict = function()
                # Dynamically import functions needed for this template file
                else:
                    functions = importlib.import_module("templatePython.functions."+template.split('/')[-1].replace('.','_'))
                    template_dict = functions.get_template_dict(self.component)
                template_object = MyTemplate(content, trimlines=False)
                template_object.identifiers()
                file_content = template_object.substitute(**template_dict)
                with open(output_file, 'w') as ostream:
                    ostream.write(file_content)

    def template_to_file_interface(self, interface_name, template, output_file):
            with open(template, 'r') as istream:
                content = istream.read()
                function_name = template.split('/')[-1].replace('.', '_')
                if hasattr(self,function_name):
                    function = getattr(self, function_name)
                    template_dict = function(self.component)
                # Dynamically import functions needed for this template file
                else:
                    functions = importlib.import_module("templatePython.functions."+template.split('/')[-1].replace('.','_'))
                    template_dict = functions.get_template_dict(self.component, interface_name)
                template_object = MyTemplate(content)
                template_object.identifiers()
                file_content = template_object.substitute(**template_dict)
                file_content = file_content.replace('<LINEREMOVE>\n','')
                with open(output_file, 'w') as ostream:
                    ostream.write(file_content)

    def README_md(self):
        return {'component_name': self.component.name}

    def DoxyFile(self):
        return {}

    def README_RCNODE_txt(self):
        return {}

    def CMakeLists_txt(self):
        if self.component.gui is not None:
            wrap_python_ui="WRAP_PYTHON_UI( mainUI )"
        else:
            wrap_python_ui = ""

        return {'wrap_python_ui': wrap_python_ui,
                'component_name': self.component.name}

    def mainUI_ui(self):
        return {'gui_type': self.component.gui.widget,
                'component_name': self.component.name}
