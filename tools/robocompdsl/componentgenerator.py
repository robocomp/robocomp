import filecmp
import os
import subprocess
import sys

import rcExceptions
import robocompdslutils
from dsl_parsers.dsl_factory import DSLFactory

sys.path.append("/opt/robocomp/python")
from dsl_parsers import dsl_factory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool
from templatepython import AbstractTemplate


FILES = {
    'python': {
        'files': [
            'CMakeLists.txt', 'DoxyFile', 'README-RCNODE.txt', 'README.md', 'etc/config', 'src/main.py',
            'src/genericworker.py', 'src/specificworker.py', 'src/mainUI.ui'
        ],
        'avoid_overwrite': [
            'src/specificworker.py', 'src/mainUI.ui', 'README.md', 'etc/config'
        ],
        'servant_files': ["SERVANT.PY"],
        'template_path': "/opt/robocomp/share/robocompdsl/templatePython/"
    },
    'cpp': {
        'files': [
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
        'template_path': "/opt/robocomp/share/robocompdsl/templateCPP/"
    }

}

LANG_TO_TEMPLATE = {
    'cpp': 'cpp',
    'cpp11': 'cpp',
    'python': 'python',
    'python3': 'python',
    'python2': 'python'
}

class ComponentGenerator:
    def __init__(self):
        self.__cdsl_file = None
        self.__output_path = None
        self.__include_dirs = None
        self.diff = None
        self.component = None
        self.imports = None

    @property
    def cdsl_file(self):
        return self.__cdsl_file

    @cdsl_file.setter
    def cdsl_file(self, value):
        assert isinstance(value, str), "cdsl_file must be a string not %s" % str(type(value))
        assert os.path.exists(value), "%s cdsl file not found." % value
        self.__cdsl_file = value

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

    def generate(self, cdsl_file, output_path, include_dirs, diff=None):
        self.cdsl_file = cdsl_file
        self.output_path = output_path
        self.include_dirs = include_dirs
        self.diff = diff
        self.load_component()
        new_existing_files = self.create_files()
        self.show_diff(new_existing_files)

    def load_component(self, diff=False):
        self.component = dsl_factory.DSLFactory().from_file(self.cdsl_file, includeDirectories=self.include_dirs)

        # TODO: '#'.join? and remane to imports_str
        self.imports = ''.join([imp + '#' for imp in self.component.imports])

        # verification
        pool = IDSLPool(self.imports, self.include_dirs)
        interface_list = self.component.requires + self.component.implements + self.component.subscribesTo + \
                         self.component.publishes

        for interface_required in interface_list:
            interface_required = interface_required if isinstance(interface_required, str) else interface_required[0]
            if not pool.moduleProviding(interface_required):
                raise rcExceptions.InterfaceNotFound(interface_required, pool.interfaces())

    def create_files(self):
        # Check output directory
        self.create_component_directories()

        # Generate specific_component
        new_existing_files = self.generate_component()


        if self.component.usingROS is True:
            for imp in self.component.imports:
                self.generate_ROS_headers(imp)

        return new_existing_files

    def show_diff(self, new_existing_files):
        # Code to launch diff tool on .new files to be compared with their old version
        if self.diff is not None and len(new_existing_files) > 0:
            diff_tool, _ = robocompdslutils.get_diff_tool(prefered=self.diff)
            print("Executing diff tool for existing files. Close if no change is needed.")
            for o_file, n_file in new_existing_files.items():
                if not filecmp.cmp(o_file, n_file):
                    print([diff_tool, o_file, n_file])
                    try:
                        subprocess.call([diff_tool, o_file, n_file])
                    except KeyboardInterrupt as e:
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

    def generate_component(self):
        language = self.component.language.lower()
        need_storm = False
        for pub in self.component.publishes + self.component.subscribesTo:
            if communication_is_ice(pub):
                need_storm = True
                break

        #
        # Generate regular files
        #
        new_existing_files = {}
        template = LANG_TO_TEMPLATE[language]
        for template_file in FILES[template]['files']:
            if template_file == 'src/mainUI.ui' and self.component.gui is None: continue
            if language == 'python' and template_file == 'CMakeLists.txt' and self.component.gui is None: continue
            if template_file == 'README-RCNODE.txt' and not need_storm: continue

            if language == 'python' and template_file == 'src/main.py':
                ofile = os.path.join(self.output_path, 'src', self.component.name + '.py')
            else:
                ofile = os.path.join(self.output_path, template_file)

            if template_file in FILES[template]['avoid_overwrite'] and os.path.exists(ofile):
                print('Not overwriting specific file "' + ofile + '", saving it to ' + ofile + '.new')
                new_existing_files[os.path.abspath(ofile)] = os.path.abspath(ofile) + '.new'
                ofile += '.new'

            ifile = os.path.join(FILES[template]['template_path'], template_file)
            print('Generating', ofile)
            params = {
                "theCDSL": self.cdsl_file,
                "theIDSLs": self.imports,
                "theIDSLPaths": '#'.join(self.include_dirs)
            }
            if template == 'python':
                template_obj = AbstractTemplate(self.component)
                template_obj.template_to_file(FILES[template]['template_path'] + template_file, ofile)
            else:
                cog_command = robocompdslutils.generate_cog_command(params, ifile, ofile)
                robocompdslutils.run_cog_and_replace_tags(cog_command, ofile)
            if language == 'python' and template_file == 'src/main.py':
                os.chmod(ofile, os.stat(ofile).st_mode | 0o111)

        #
        # Generate interface-dependent files
        #
        for interface in self.component.implements + self.component.subscribesTo:
            if isinstance(interface, (list, tuple)):
                interface_name = interface[0]
            else:
                interface_name = interface
            if communication_is_ice(interface):
                for template_file in FILES[template]['servant_files']:
                    ofile = os.path.join(self.output_path, 'src', interface_name.lower() + 'I.' + template_file.split('.')[
                        -1].lower())
                    print('Generating %s (servant for %s)' %    (ofile, interface_name))
                    # Call cog
                    params = {
                        "theCDSL": self.cdsl_file,
                        "theIDSLs": self.imports,
                        "theIDSLPaths": '#'.join(self.include_dirs),
                        "theInterface": interface_name
                    }
                    # TODO: remove after testing new templates
                    if template == 'python':
                        template_obj = AbstractTemplate(self.component)
                        template_obj.template_to_file_interface(interface_name, FILES[template]['template_path'] + template_file,
                                                      ofile)
                    else:
                        cog_command = robocompdslutils.generate_cog_command(params,
                                                                            FILES[template][
                                                                                'template_path'] + template_file,
                                                                            ofile)
                        robocompdslutils.run_cog_and_replace_tags(cog_command, ofile)

        return new_existing_files

    def generate_ROS_headers(self, idsl_file):
        """
        :param idsl_file: is the IDSL file imported in the CDSL
        :return:
        """
        imported = []
        idsl = DSLFactory().from_file(idsl_file, includeDirectories=self.include_dirs)
        if not os.path.exists(self.output_path):
            robocompdslutils.create_directory(self.output_path)

        def generarH(idslFile, imported):
            idsl = DSLFactory().from_file(idslFile)
            try:
                os.system("rm -f " + self.output_path + "/" + idsl['module']['name'] + "ROS/msg/__init__.py")
                os.system("rm -f " + self.output_path + "/" + idsl['module']['name'] + "ROS/srv/__init__.py")
            except KeyError:
                print("No module found in %s" % idsl_file)
            for imp in idsl['structs'] + idsl['sequences']:
                if imp['type'] in ['struct', 'sequence']:
                    for f in ["SERVANT.MSG"]:
                        ofile = self.output_path + "/" + imp['name'] + "." + f.split('.')[-1].lower()
                        print('Generating', ofile, ' (servant for', idslFile.split('.')[0].lower() + ')')

                        ofile_dir = os.path.dirname(ofile)
                        if not os.path.exists(ofile_dir):
                            os.makedirs(ofile_dir)
                        # Call cog
                        params = {
                            "theIDSLPaths": '#'.join(self.include_dirs),
                            "structName": imp['name'],
                            "theIDSL": idslFile
                        }
                        cog_command = robocompdslutils.generate_cog_command(params, "/opt/robocomp/share/robocompdsl/templateCPP/" + f,
                                                           ofile)
                        robocompdslutils.run_cog_and_replace_tags(cog_command, ofile)
                        commandCPP = "/opt/ros/melodic/lib/gencpp/gen_cpp.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -I" + \
                                     idsl['name'] + "ROS:" + self.output_path
                        commandPY = "/opt/ros/melodic/lib/genpy/genmsg_py.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -I" + \
                                    idsl['name'] + "ROS:" + self.output_path
                        for impo in imported:
                            if not impo == idsl['module']['name'] + "ROS":
                                commandCPP = commandCPP + " -I" + impo + ":" + self.output_path
                                commandPY = commandPY + " -I" + impo + ":" + self.output_path
                        commandCPP = commandCPP + " -p " + idsl['name'] + "ROS -o " + self.output_path + "/" + idsl[
                            'name'] + "ROS -e /opt/ros/melodic/share/gencpp"
                        commandPY = commandPY + " -p " + idsl['name'] + "ROS -o " + self.output_path + "/" + idsl[
                            'name'] + "ROS/msg"
                        if self.component.language.lower() == 'cpp':
                            os.system(commandCPP)
                        else:
                            os.system(commandPY)
                        try:
                            fileInit = open(self.output_path + "/" + idsl['name'] + "ROS/msg/__init__.py", 'a')
                            fileInit.write("from ._" + imp['name'] + " import *\n")
                            fileInit.close()
                        except:
                            pass
            for imp in idsl['interfaces']:
                for ima in [self.component.implements + self.component.requires]:
                    im = ima
                    if type(im) != type(''):
                        im = im[0]
                    if not communication_is_ice(ima) and im == imp['name']:
                        for method in imp['methods'].values():
                            if 'params' in method:
                                if len(method['params']) == 2:
                                    for f in ["SERVANT.SRV"]:
                                        ofile = self.output_path + "/" + method['name'] + "." + f.split('.')[-1].lower()
                                        print('Generating', ofile, ' (servant for',
                                              idslFile.split('.')[0].lower() + ')')
                                        # Call cog
                                        params = {
                                            "theIDSLPaths": '#'.join(self.include_dirs),
                                            "methodName": method['name'],
                                            "theIDSL": idslFile
                                        }
                                        cog_command = robocompdslutils.generate_cog_command(params,
                                                                           "/opt/robocomp/share/robocompdsl/templateCPP/" + f,
                                                                           ofile)
                                        robocompdslutils.run_cog_and_replace_tags(cog_command, ofile)
                                        commandCPP = "/opt/ros/melodic/lib/gencpp/gen_cpp.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -Istd_srvs:/opt/ros/melodic/share/std_srv/cmake/../srv -I" + \
                                                     idsl['module']['name'] + "ROS:" + self.output_path
                                        commandPY = "/opt/ros/melodic/lib/genpy/gensrv_py.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -Istd_srvs:/opt/ros/kinetic/share/std_srv/cmake/../srv -I" + \
                                                    idsl['module']['name'] + "ROS:" + self.output_path
                                        for impo in imported:
                                            if not impo == idsl['module']['name'] + "ROS":
                                                commandCPP = commandCPP + " -I" + impo + ":" + self.output_path
                                                commandPY = commandPY + " -I" + impo + ":" + self.output_path

                                        commandCPP = commandCPP + " -p " + idsl['module'][
                                            'name'] + "ROS -o " + self.output_path + "/" + idsl['module'][
                                                         'name'] + "ROS -e /opt/ros/melodic/share/gencpp/cmake/.."
                                        commandPY = commandPY + " -p " + idsl['module'][
                                            'name'] + "ROS -o " + self.output_path + "/" + idsl['module']['name'] + "ROS/srv"
                                        if self.component.language.lower() == 'cpp':
                                            os.system(commandCPP)
                                        else:
                                            os.system(commandPY)
                                        try:
                                            fileInit = open(
                                                self.output_path + "/" + idsl['module']['name'] + "ROS/srv/__init__.py", 'a')
                                            fileInit.write("from ._" + method['name'] + " import *\n")
                                            fileInit.close()
                                        except:
                                            pass
                                else:
                                    for param in enumerate(method['params']):
                                        print(param[0], '-->', param[1])
                                    raise IndexError(
                                        "ERROR: ROS service with incorrect number of parameters. ROS only supports remote procedure calls of the form: void method(type inVar, out type outVar);")
                            else:
                                raise KeyError(
                                    "ERROR: service without params. Form is: void method(type inVar, out type outVar);")

            os.system("touch " + self.output_path + "/" + idsl['module']['name'] + "ROS/__init__.py")
            return idsl['module']['name'] + "ROS"

        try:
            for importIDSL in idsl['imports'] + idsl['recursive_imports']:
                imported.append(generarH("/opt/robocomp/interfaces/IDSLs/" + importIDSL, []))
        except:
            pass

        generarH(idsl_file, imported)
        os.system("rm " + self.output_path + "/*.msg")
        os.system("rm " + self.output_path + "/*.srv")

    def create_component_directories(self):
        if not os.path.exists(self.output_path):
            robocompdslutils.create_directory(self.output_path)
        # Create directories within the output directory
        new_dirs = ["bin", "src", "etc"]
        for new_dir in new_dirs:
            if self.component.language.lower() == "python" and new_dir == "bin": continue
            try:
                robocompdslutils.create_directory(os.path.join(self.output_path, new_dir))
            except:
                print('There was a problem creating a directory %s' % new_dir)
                raise

