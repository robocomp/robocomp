import datetime
import importlib
from collections import ChainMap
from string import Template

from templates.templateCPP.functions import servant


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



class TemplateCpp:
    def __init__(self, component):
        self.component = component

    def template_to_file(self, template, output_file):
            with open(template, 'r') as istream:
                content = istream.read()
                function_name = template.split('/')[-1].replace('.', '_').replace('-','_')
                if hasattr(self,function_name):
                    function = getattr(self, function_name)
                    template_dict = function(self.component)
                # Dynamically import functions needed for this template file
                else:
                    functions_file = template[template.find("templateCPP/files")+len("templateCPP/files"):].replace('.', '_').replace('/','.')
                    functions = importlib.import_module("templates.templateCPP.functions"+functions_file)
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
                    template_dict = function(interface_name, self.component)
                # Dynamically import functions needed for this template file
                else:
                    functions_file = template.replace("templateCPP/functions/", "").replace('.', '_').replace('/','.')
                    functions = importlib.import_module("templateCPP.functions." + functions_file)
                    template_dict = functions.get_template_dict(self.component, interface_name)
                template_object = MyTemplate(content)
                template_object.identifiers()
                file_content = template_object.substitute(**template_dict)
                with open(output_file, 'w') as ostream:
                    ostream.write(file_content)

    def SERVANT_H(self, interface_name, component):
        module = component.idsl_pool.moduleProviding(interface_name)
        return {
            'year': str(datetime.date.today().year),
            'interface_name': interface_name,
            'interface_name_upper': interface_name.upper(),
            'filename_without_extension': module['filename'].split('/')[-1].split('.')[0],
            'module_name': module['name'],
            'interface_methods_definition': servant.interface_methods_definition(component,
                                                                                 module,
                                                                                 interface_name)
        }

    def SERVANT_CPP(self, interface_name, component):
        return {
            'year': str(datetime.date.today().year),
            'interface_name': interface_name,
            'interface_name_lower': interface_name.lower(),
            'interface_methods_creation': servant.interface_methods_creation(component, interface_name)
        }

    def README_md(self, component):
        return {
            'component_name': component.name
        }

    def DoxyFile(self, component):
        return {
            'component_name': component.name
        }

    def CMakeLists_txt(self, component):
        return {
            'component_name': component.name
        }
