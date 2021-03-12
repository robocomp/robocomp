import filecmp
import os
import subprocess
import sys

import robocompdslutils
from templates.templateCPP.templatecpp import TemplatesManagerCpp
from templates.templateICE.templateice import TemplateManagerIce
from templates.templatePython.templatepython import TemplatesManagerPython

sys.path.append("/opt/robocomp/python")
from dsl_parsers import dsl_factory

LANG_TO_TEMPLATE = {
    'cpp': 'cpp',
    'cpp11': 'cpp',
    'python': 'python',
    'python3': 'python',
    'python2': 'python'
}


class FilesGenerator:
    def __init__(self):
        self.__dsl_file = None
        self.__output_path = None
        self.__include_dirs = None
        self.diff = None
        self.ast = None

    @property
    def dsl_file(self):
        return self.__dsl_file

    @dsl_file.setter
    def dsl_file(self, value):
        assert isinstance(value, str), "dsl_file must be a string not %s" % str(type(value))
        assert os.path.exists(value), "%s cdsl file not found." % value
        self.__dsl_file = value

    @property
    def output_path(self):
        return self.__output_path

    @output_path.setter
    def output_path(self, value):
        assert isinstance(value, str), "output_path must be a string not %s" % str(type(value))
        self.__output_path = value

    @property
    def include_dirs(self):
        return self.__include_dirs

    @include_dirs.setter
    def include_dirs(self, value):
        assert isinstance(value, list), "include_dirs must be a string not %s" % str(type(value))
        self.__include_dirs = value

    def generate(self, input_file, output_path, include_dirs, diff=None, test=False):
        self.dsl_file = input_file
        self.output_path = output_path
        self.include_dirs = include_dirs
        self.diff = diff
        self.__load_ast()
        new_existing_files = self.__create_files(test)
        self.__show_diff(new_existing_files)

    def __load_ast(self):
        try:
            self.ast = dsl_factory.DSLFactory().from_file(self.dsl_file, includeDirectories=self.include_dirs)
        except ValueError as e:
            print(f"Parsing error in file {rich.Text(self.dsl_file, style='red')} while generating AST.")
            print(f"Exception info: {rich.Text(e.args[0], style='red')} in line {e.args[1]} of:\n{rich.Text(e.args[2].rstrip(), style='magenta')}")
            exit(1)

    def __create_files(self, test=False):
        new_existing_files = {}
        if self.dsl_file.endswith(".cdsl") or self.dsl_file.endswith(".jcdsl"):
            # Check output directory
            self.__create_component_directories(test)

            # Generate specific_component
            new_existing_files = self.__generate_component(test)

        elif self.dsl_file.endswith(".idsl"):
            new_existing_files = self.__generate_interface()
        return new_existing_files

    def __show_diff(self, new_existing_files):
        # Code to launch diff tool on .new files to be compared with their old version
        if self.diff is not None and len(new_existing_files) > 0:
            diff_tool, _ = robocompdslutils.get_diff_tool(prefered=self.diff)
            print("Executing diff tool for existing files. Close if no change is needed.")
            for o_file, n_file in new_existing_files.items():
                if not filecmp.cmp(o_file, n_file):
                    print([diff_tool, o_file, n_file])
                    try:
                        subprocess.call([diff_tool, o_file, n_file])
                    except KeyboardInterrupt:
                        print("Comparision interrupted. All files have been generated. Check this .new files manually:")
                        for o_file2, n_file2 in new_existing_files.items():
                            if not filecmp.cmp(o_file2, n_file2):
                                print("%s %s" % (o_file2, n_file2))
                        break
                    except Exception as e:
                        print("Exception trying to execute %s" % diff_tool)
                        print(e.message)

                else:
                    print("Binary equal files %s and %s" % (o_file, n_file))

    def __generate_component(self, test):
        language = self.ast.language.lower()

        template = LANG_TO_TEMPLATE[language]
        # TODO: Template objects could be moved to a TemplateFactory
        if template == 'python':
            template_obj = TemplatesManagerPython(self.ast)
        else:
            template_obj = TemplatesManagerCpp(self.ast)

        try:
            new_existing_files = template_obj.generate_files(self.output_path)
            if template == 'python' and test:
                self.ast.requires, self.ast.implements = self.ast.implements, self.ast.requires
                self.ast.publishes, self.ast.subscribesTo = self.ast.subscribesTo, self.ast.publishes
                test_template_object = TemplatesManagerPython(self.ast)
                test_template_object.generate_files(self.output_path+"/test")
        except KeyError as e:
            print(e)
            raise

    # for module in self.ast.idsl_pool.modulePool.values():
        #     template_obj = TemplateManagerIce(module)
        #     ice_directory = os.path.join(self.output_path, "ice_files")
        #     ice_new_existing_files = template_obj.generate_files(ice_directory)
        #     new_existing_files.update(ice_new_existing_files)
        return new_existing_files

    def __generate_interface(self):
        template_obj = TemplateManagerIce(self.ast)
        new_existing_files = template_obj.generate_files(self.output_path)
        return new_existing_files

    def __create_component_directories(self, test=False):
        if not os.path.exists(self.output_path):
            robocompdslutils.create_directory(self.output_path)

        # Create directories within the output directory
        new_dirs = ["bin", "src", "etc"]
        for new_dir in new_dirs:
            if self.ast.language.lower() == "python" and new_dir == "bin":
                continue
            robocompdslutils.create_directory(os.path.join(self.output_path, new_dir))
        if test:
            test_output_path = os.path.join(self.output_path, "test")
            robocompdslutils.create_directory(test_output_path)
            for new_dir in new_dirs:
                if self.ast.language.lower() == "python" and new_dir == "bin":
                    continue
                robocompdslutils.create_directory(os.path.join(test_output_path, new_dir))
