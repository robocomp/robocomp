import importlib
import os
from abc import ABC
from collections import ChainMap
from string import Template

from dsl_parsers.parsing_utils import communication_is_ice
import rich

console = rich.console.Console()

class CustomTemplate(Template):
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
        super(CustomTemplate, self).__init__(template)
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
                if len(lines) > 0:
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
        return substituted.replace('<LINEREMOVE>\n', '')

    def identifiers(self):
        identifiers = []
        results = self.pattern.findall(self.template)
        for result in results:
            if result[3] != '' and result[3] not in identifiers:
                identifiers.append(result[3])
        return identifiers


TEMPLATES_DIR = '/opt/robocomp/python/robocompdsl/templates/'
ALT_TEMPLATES_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')


class AbstractTemplatesManager(ABC):
    def __init__(self, ast, plugins):
        self.ast = ast
        self.plugins = plugins
        super(AbstractTemplatesManager, self).__init__()

    def generate_files(self, output_path):
        pass

    def _pre_generation_check(self, template_file):
        return False

    def _output_file_rename(self, output_path, template_file):
        return os.path.join(output_path, template_file)

    def _post_generation_action(self, template_file, ofile):
        pass

    def _template_to_file(self, template, output_file, interface_name=None):
        try:
            istream = open(template, 'r')
        except FileNotFoundError:
            new_template_path = template.replace(TEMPLATES_DIR, ALT_TEMPLATES_DIR)
            istream = open(new_template_path, 'r')
        content = istream.read()
        istream.close()
        template_dict = self._get_template_dict(template, interface_name)
        template_object = CustomTemplate(content, trimlines=False)
        try:
            file_content = template_object.substitute(**template_dict)
        except KeyError as e:
            raise KeyError(f"Template keyword {str(e)} not found for Template file {template}")

        with open(output_file, 'w') as ostream:
            ostream.write(file_content)

    def _get_template_dict(self, template, interface_name=None):
        template_dict = {}
        full_path = os.path.join(TEMPLATES_DIR, self.files['template_path'])
        template_name = template.replace(full_path, "")
        # look for a method in the class with the name of the file
        for plugin in self.plugins:
            new_template_dict = plugin.get_template_dict(template_name, self.ast, interface_name)
            for entry, value in new_template_dict.items():
                if entry in template_dict:
                    template_dict[entry] += new_template_dict[entry]
                else:
                    template_dict[entry] = new_template_dict[entry]
        return template_dict


class ComponentTemplatesManager(AbstractTemplatesManager):
    def __init__(self, component, plugins):
        super(ComponentTemplatesManager, self).__init__(component, plugins)

    def generate_files(self, output_path):
        #
        # Generate regular files
        #
        new_existing_files = {}
        for template_file in self.files['regular']:
            if self._pre_generation_check(template_file): continue
            if template_file == 'README-RCNODE.txt' and not self._need_storm(): continue
            if template_file == 'src/mainUI.ui' and self.ast.gui is None: continue

            ofile = self._output_file_rename(output_path, template_file)

            if template_file in self.files['avoid_overwrite'] and os.path.exists(ofile):
                console.print(':eye:  Not overwriting specific file "' + ofile + '", saving it to ' + ofile + '.new', style='yellow')
                new_existing_files[os.path.abspath(ofile)] = os.path.abspath(ofile) + '.new'
                ofile += '.new'

            ifile = os.path.join(TEMPLATES_DIR, self.files['template_path'], template_file)
            console.print(f":thumbs_up: Generating {ofile}", style='green')
            try:
                self._template_to_file(ifile, ofile)
            except ValueError as e:
                console.print(e)
            self._post_generation_action(template_file, ofile)

        for interface in self.ast.implements + self.ast.subscribesTo:
            if communication_is_ice(interface):
                for template_file in self.files['servant_files']:
                    ofile = os.path.join(output_path, 'src', interface.name.lower() + 'I.' + template_file.split('.')[
                        -1].lower())
                    console.print(':thumbs_up: Generating %s (servant for %s)' % (ofile, interface.name), style='green')
                    ifile = os.path.join(TEMPLATES_DIR, self.files['template_path'], template_file)
                    self._template_to_file(ifile, ofile, interface.name)
        return new_existing_files

    def _output_file_rename(self, output_path, template_file):
        return os.path.join(output_path, template_file)

    def _need_storm(self):
        for pub in self.ast.publishes + self.ast.subscribesTo:
            if communication_is_ice(pub):
                return True
        return False

    @property
    def component(self):
        return self.ast


class InterfaceTemplateManager(AbstractTemplatesManager):
    def __init__(self, interface, plugins):
        super(InterfaceTemplateManager, self).__init__(interface, plugins)

    def generate_files(self, output_path):
        #
        # Generate regular files
        #
        new_existing_files = {}
        for template_file in self.files['regular']:
            if output_path.endswith('.ice'):
                pass
            elif os.path.exists(output_path):
                if self.module['filename']:
                    new_filename = self.module['filename'].split('/')[-1].split('.')[0] + ".ice"
                    output_path = os.path.join(output_path, new_filename)
                elif os.path.exists(output_path) and self.module['name']:
                    output_path = os.path.join(output_path, self.module['name'] + ".ice")
                else:
                    raise ValueError("Invalid output name in module for the ice interface")
            else:
                raise ValueError("Invalid output path for the ice interface")
            ofile = self._output_file_rename(output_path, template_file)

            # if os.path.exists(ofile):
            #     print('Not overwriting specific file "' + ofile + '", saving it to ' + ofile + '.new')
            #     new_existing_files[os.path.abspath(ofile)] = os.path.abspath(ofile) + '.new'
            #     ofile += '.new'

            ifile = os.path.join(TEMPLATES_DIR, self.files['template_path'], template_file)
            console.print(f":thumbs_up: Generating {ofile}", style='green')
            self._template_to_file(ifile, ofile)

            self._post_generation_action(template_file, ofile)

        return new_existing_files

    def _output_file_rename(self, output_path, template_file):
        return output_path

    @property
    def module(self):
        return self.ast
