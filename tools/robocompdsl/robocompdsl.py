#!/usr/bin/env python3

# TODO
#
# Read ports from component-ports.txt for the files in etc.
#
#
import argparse
import filecmp
import os
import subprocess
import sys
from distutils import spawn

from cogapp import Cog

# from parseCDSL import *
sys.path.append("/opt/robocomp/python")
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool
import rcExceptions
import sys

DIFF_TOOLS = ["meld", "kdiff3", "diff"]

DUMMY_CDSL_STRING = """import "import1.idsl";
import "import2.idsl";

Component <CHANGETHECOMPONENTNAME>
{
    Communications
    {
        implements interfaceName;
        requires otherName;
        subscribesTo topicToSubscribeTo;
        publishes topicToPublish;
    };
    language Cpp//Cpp11//python;
    gui Qt(QWidget//QDialog//QMainWindow);
    //options agmagent, InnerModelViewer;
    statemachine "statemachine.smdsl";
};\n\n"""

DUMMY_SMDSL_STRING = """
/* CHANGE THE NAME OF THE MACHINE IF YOU MAKE
   ANY CHANGE TO THE DEFAULT STATES OR TRANSITIONS */

defaultMachine{
    states compute;
    initial_state initialize;
    end_state finalize;
    transitions{
        initialize => compute;
        compute => compute;
        compute => finalize;
    };
};


/* --------------------------------------------------------------
   This is the accepted syntax for the State Machine definition 

name_machine{
    [states name_state *[, name_state];]
    [initial_state name_state;]
    [end_state name_state;]
    [transitions{
        name_state => name_state *[, name_state];
        *[name_state => name_state *[, name_state];]
    };]
};

[:parent_state [parallel]{
    states name_state *[, name_state];
    [initial_state name_state;]
    [end_state name_state;]
    [transitions{
        name_state => name_state *[, name_state];
        *[name_state => name_state *[, name_state];]
    };]
};]

------------------------------------------------------------------ */\n"""



def generate_ROS_headers(idsl_file, output_path, comp, include_directories):
    """

    :param idsl_file: is the IDSL file imported in the CDSL, outputPath is the path where the ROS headers are to be generated
    :param output_path:
    :param comp:
    :param include_directories:
    :return:
    """
    imported = []
    idsl = DSLFactory().from_file(idsl_file, includeDirectories=include_directories)
    if not os.path.exists(output_path):
        create_directory(output_path)

    def generarH(idslFile, imported):
        idsl = DSLFactory().from_file(idslFile)
        try:
            os.system("rm -f " + output_path + "/" + idsl['module']['name'] + "ROS/msg/__init__.py")
            os.system("rm -f " + output_path + "/" + idsl['module']['name'] + "ROS/srv/__init__.py")
        except KeyError:
            print("No module found in %s" % idsl_file)
        for imp in idsl['structs']+idsl['sequences']:
            if imp['type'] in ['struct','sequence']:
                for f in [ "SERVANT.MSG"]:
                    ofile = output_path + "/" + imp['name'] + "." + f.split('.')[-1].lower()
                    print('Generating', ofile, ' (servant for', idslFile.split('.')[0].lower() + ')')

                    ofile_dir = os.path.dirname(ofile)
                    if not os.path.exists(ofile_dir):
                        os.makedirs(ofile_dir)
                    # Call cog
                    params = {
                        "theIDSLPaths": '#'.join(include_directories),
                        "structName": imp['name'],
                        "theIDSL": idslFile
                    }
                    cog_command = generate_cog_command(params, "/opt/robocomp/share/robocompdsl/templateCPP/" + f, ofile)
                    run_cog_and_replace_tags(cog_command, ofile)
                    commandCPP = "/opt/ros/melodic/lib/gencpp/gen_cpp.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -I" + idsl['name'] + "ROS:" + output_path
                    commandPY  = "/opt/ros/melodic/lib/genpy/genmsg_py.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -I" + idsl['name'] + "ROS:" + output_path
                    for impo in imported:
                        if not impo == idsl['module']['name']+"ROS":
                            commandCPP = commandCPP + " -I" + impo + ":" + output_path
                            commandPY  = commandPY + " -I" + impo + ":" + output_path
                    commandCPP = commandCPP + " -p " + idsl['name'] + "ROS -o " + output_path + "/" + idsl['name'] + "ROS -e /opt/ros/melodic/share/gencpp"
                    commandPY = commandPY + " -p " + idsl['name'] + "ROS -o " + output_path + "/" + idsl['name'] + "ROS/msg"
                    if comp['language'].lower() == 'cpp':
                        os.system(commandCPP)
                    else:
                        os.system(commandPY)
                    try:
                        fileInit = open(output_path + "/" + idsl['name'] + "ROS/msg/__init__.py", 'a')
                        fileInit.write("from ._"+imp['name']+" import *\n")
                        fileInit.close()
                    except:
                        pass
        for imp in idsl['interfaces']:
                for ima in [comp['implements']+comp['requires']]:
                    im = ima
                    if type(im) != type(''):
                        im = im[0]
                    if not communication_is_ice(ima) and im == imp['name']:
                        for method in imp['methods'].values():
                            if 'params' in method:
                                if len(method['params']) == 2:
                                    for f in ["SERVANT.SRV"]:
                                        ofile = output_path + "/" + method['name'] + "." + f.split('.')[-1].lower()
                                        print('Generating', ofile, ' (servant for', idslFile.split('.')[0].lower() + ')')
                                        # Call cog
                                        params = {
                                            "theIDSLPaths": '#'.join(include_directories),
                                            "methodName": method['name'],
                                            "theIDSL": idslFile
                                        }
                                        cog_command = generate_cog_command(params, "/opt/robocomp/share/robocompdsl/templateCPP/" + f, ofile)
                                        run_cog_and_replace_tags(cog_command, ofile)
                                        commandCPP = "/opt/ros/melodic/lib/gencpp/gen_cpp.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -Istd_srvs:/opt/ros/melodic/share/std_srv/cmake/../srv -I" + idsl['module']['name'] + "ROS:" + output_path
                                        commandPY  = "/opt/ros/melodic/lib/genpy/gensrv_py.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -Istd_srvs:/opt/ros/kinetic/share/std_srv/cmake/../srv -I" + idsl['module']['name'] + "ROS:" + output_path
                                        for impo in imported:
                                            if not impo == idsl['module']['name']+"ROS":
                                                commandCPP = commandCPP + " -I" + impo + ":" + output_path
                                                commandPY  = commandPY + " -I" + impo + ":" + output_path

                                        commandCPP = commandCPP + " -p " + idsl['module']['name'] + "ROS -o " + output_path + "/" + idsl['module']['name'] + "ROS -e /opt/ros/melodic/share/gencpp/cmake/.."
                                        commandPY = commandPY + " -p " + idsl['module']['name'] + "ROS -o " + output_path + "/" + idsl['module']['name'] + "ROS/srv"
                                        if comp['language'].lower() == 'cpp':
                                            os.system(commandCPP)
                                        else:
                                            os.system(commandPY)
                                        try:
                                            fileInit = open(output_path + "/" + idsl['module']['name'] + "ROS/srv/__init__.py", 'a')
                                            fileInit.write("from ._"+method['name']+" import *\n")
                                            fileInit.close()
                                        except:
                                            pass
                                else:
                                    for param in enumerate(method['params']):
                                        print(param[0], '-->', param[1])
                                    raise IndexError("ERROR: ROS service with incorrect number of parameters. ROS only supports remote procedure calls of the form: void method(type inVar, out type outVar);")
                            else:
                                raise KeyError("ERROR: service without params. Form is: void method(type inVar, out type outVar);")

        os.system("touch " + output_path + "/" + idsl['module']['name'] + "ROS/__init__.py")
        return idsl['module']['name']+"ROS"
    try:
        for importIDSL in idsl['imports']+idsl['recursive_imports']:
            imported.append(generarH("/opt/robocomp/interfaces/IDSLs/"+importIDSL, []))
    except:
        pass

    generarH(idsl_file, imported)
    os.system("rm " + output_path + "/*.msg")
    os.system("rm " + output_path + "/*.srv")
#
# Misc functions
#
def replaceTagsInFile(path):
    i = open(path, 'r')
    text = i.read()
    reps = []
    reps.append(["\n<@@<" ,""])
    reps.append([">@@>\n" ,""])
    reps.append(["<TABHERE>", '\t'])
    reps.append(["<S1>", ' '])
    reps.append(["<S2>", '  '])
    reps.append(["<S4>", '    '])
    for r in reps:
        text = text.replace(r[0], r[1])
    i.close()
    w = open(path, 'w')
    w.write(text)
    w.close()

def generateDummyCDSL(path):
    if os.path.exists(path):
        print("File", path, "already exists.\nExiting...")
    else:
        print("Generating dummy CDSL file:", path)

        name = path.split('/')[-1].split('.')[0]
        string = DUMMY_CDSL_STRING.replace('<CHANGETHECOMPONENTNAME>', name)
        open(path, "w").write(string)


def generateDummySMDSL(path):
    if os.path.exists(path):
        print("File", path, "already exists.\nExiting...")
    else:
        print("Generating dummy SMDSL file:", path)

        open(path, "w").write(DUMMY_SMDSL_STRING)

def get_diff_tool(prefered=None):
    if prefered in DIFF_TOOLS:
        tool_path = spawn.find_executable(prefered)
        if tool_path is not "":
            return prefered, tool_path
    for tool in DIFF_TOOLS:
        tool_path = spawn.find_executable(tool)
        if tool_path is not "":
            return tool, tool_path
    return None, None

#Class deffining colors for terminal printing
class BColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class MyParser(argparse.ArgumentParser):
    """
    Class to print(colored error message on argparse
    """
    def error(self, message):
        sys.stderr.write((BColors.FAIL + 'error: %s\n' + BColors.ENDC) % message)
        self.print_help()
        sys.exit(2)


class FullPaths(argparse.Action):
    """
    Class to expand and check list of paths arguments in argparse
    """
    def __call__(self, parser, namespace, values, option_string=None):
        dirnames = []
        for value in values:
            dirname = os.path.abspath(os.path.expanduser(value))
            if not os.path.isdir(dirname):
                msg = "{0} is not a directory".format(dirname)
                raise argparse.ArgumentTypeError(msg)
            else:
                dirnames.append(dirname)

        else:
            setattr(namespace, self.dest, dirnames)


def generate_component_from_cdsl(cdsl_file, output_path, include_dirs, diff=False):
    component = DSLFactory().from_file(cdsl_file, includeDirectories=include_dirs)
    imports = ''.join([imp + '#' for imp in component['imports']])

    # verification
    pool = IDSLPool(imports, include_dirs)
    interface_list = component['requires'] + component['implements'] + component['subscribesTo'] + component[
        'publishes']

    for interface_required in interface_list:
        interface_required = interface_required if isinstance(interface_required, str) else interface_required[0]
        if not pool.moduleProviding(interface_required):
            raise rcExceptions.InterfaceNotFound(interface_required, pool.interfaces())

    #
    # Check output directory
    #
    create_component_directories(output_path, component["language"])

    # Generate specific_component
    if component['language'].lower() == 'cpp' or component['language'].lower() == 'cpp11':
        new_existing_files = generate_cpp_component(component, cdsl_file, output_path, include_dirs, imports)
    elif component['language'].lower() == 'python':
        new_existing_files = generate_python_component(component, cdsl_file, output_path, include_dirs, imports)
    else:
        print('Unsupported language', component['language'])
        sys.exit(-1)

    if component['usingROS'] is True:
        for imp in component['imports']:
            generate_ROS_headers(imp, output_path + "/src", component, include_dirs)

    # Code to launch diff tool on .new files to be compared with their old version
    if diff is not None and len(new_existing_files) > 0:
        diff_tool, _ = get_diff_tool(prefered=diff)
        print("Executing diff tool for existing files. Close if no change is needed.")
        for o_file, n_file in new_existing_files.items():
            if not filecmp.cmp(o_file, n_file):
                print([diff_tool, o_file, n_file])
                try:
                    subprocess.call([diff_tool, o_file, n_file])
                except KeyboardInterrupt as e:
                    print("Comparasion interrupted. All files have been generated. Check this .new files manually:")
                    for o_file2, n_file2 in new_existing_files.items():
                        if not filecmp.cmp(o_file2, n_file2):
                            print("%s %s" % (o_file2, n_file2))
                    break
                except Exception as e:
                    print("Exception trying to execute %s" % diff_tool)
                    print(e.message)

            else:
                print("Binary equal files %s and %s" % (o_file, n_file))


def generate_python_component(component, cdsl_file, output_path, include_dirs, imports):
    need_storm = False
    for pub in component['publishes']:
        if communication_is_ice(pub):
            need_storm = True
    for sub in component['subscribesTo']:
        if communication_is_ice(sub):
            need_storm = True
    #
    # Generate regular files
    #
    files = ['CMakeLists.txt', 'DoxyFile', 'README-STORM.txt', 'README.md', 'etc/config', 'src/main.py',
             'src/genericworker.py', 'src/specificworker.py', 'src/mainUI.ui']
    specific_files = ['src/specificworker.py', 'src/mainUI.ui', 'README.md', 'etc/config']

    new_existing_files = {}
    for template_file in files:
        if template_file == 'src/mainUI.ui' and component['gui'] is None: continue
        if template_file == 'CMakeLists.txt' and component['gui'] is None: continue
        if template_file == 'README-STORM.txt' and need_storm is False: continue

        if template_file == 'src/main.py':
            ofile = output_path + '/src/' + component['name'] + '.py'
        else:
            ofile = output_path + '/' + template_file

        if template_file in specific_files and os.path.exists(ofile):
            print('Not overwriting specific file "' + ofile + '", saving it to ' + ofile + '.new')
            new_existing_files[os.path.abspath(ofile)] = os.path.abspath(ofile) + '.new'
            ofile += '.new'

        ifile = "/opt/robocomp/share/robocompdsl/templatePython/" + template_file
        print('Generating', ofile)
        params = {
            "theCDSL": cdsl_file,
            "theIDSLs": imports,
            "theIDSLPaths": '#'.join(include_dirs)
        }
        cog_command = generate_cog_command(params, ifile, ofile)
        run_cog_and_replace_tags(cog_command, ofile)
        if template_file == 'src/main.py':
            os.chmod(ofile, os.stat(ofile).st_mode | 0o111)

    #
    # Generate interface-dependent files
    #
    for imp in component['implements'] + component['subscribesTo']:
        if not isinstance(imp, str):
            im = imp[0]
        else:
            im = imp
        if communication_is_ice(imp):
            for template_file in ["SERVANT.PY"]:
                ofile = output_path + '/src/' + im.lower() + 'I.' + template_file.split('.')[-1].lower()
                print('Generating ', ofile, ' (servant for', im + ')')
                # Call cog
                params = {
                    "theCDSL": cdsl_file,
                    "theIDSLs": imports,
                    "theIDSLPaths": '#'.join(include_dirs),
                    "theInterface": im
                }
                cog_command = generate_cog_command(params, "/opt/robocomp/share/robocompdsl/templatePython/"+template_file, ofile)
                run_cog_and_replace_tags(cog_command, ofile)
    return new_existing_files


def generate_cpp_component(component, cdsl_file, output_path, include_dirs, imports):
    #
    # Check output directory
    #
    create_component_directories(output_path, component['language'])

    #
    # Generate regular files
    #
    files = ['CMakeLists.txt', 'DoxyFile', 'README-STORM.txt', 'README.md', 'etc/config', 'src/main.cpp',
             'src/CMakeLists.txt', 'src/CMakeListsSpecific.txt', 'src/commonbehaviorI.h', 'src/commonbehaviorI.cpp',
             'src/genericmonitor.h', 'src/genericmonitor.cpp', 'src/config.h', 'src/specificmonitor.h',
             'src/specificmonitor.cpp', 'src/genericworker.h', 'src/genericworker.cpp', 'src/specificworker.h',
             'src/specificworker.cpp', 'src/mainUI.ui']
    specific_files = ['src/specificworker.h', 'src/specificworker.cpp', 'src/CMakeListsSpecific.txt',
                      'src/mainUI.ui', 'src/specificmonitor.h', 'src/specificmonitor.cpp', 'README.md', 'etc/config']
    new_existing_files = {}
    for template_file in files:
        ofile = output_path + '/' + template_file
        if template_file in specific_files and os.path.exists(ofile):
            print('Not overwriting specific file "' + ofile + '", saving it to ' + ofile + '.new')
            new_existing_files[os.path.abspath(ofile)] = os.path.abspath(ofile) + '.new'
            ofile += '.new'
        ifile = "/opt/robocomp/share/robocompdsl/templateCPP/" + template_file
        if template_file != 'src/mainUI.ui' or component['gui'] is not None:
            print('Generating', ofile)

            params = {
                "theCDSL": cdsl_file,
                "theIDSLs": imports,
                "theIDSLPaths": '#'.join(include_dirs)
            }
            cog_command = generate_cog_command(params, ifile, ofile)
            run_cog_and_replace_tags(cog_command, ofile)
    #
    # Generate interface-dependent files
    #
    for ima in component['implements']:
        iface_name = ima
        if not isinstance(iface_name, str):
            iface_name = iface_name[0]
        if communication_is_ice(ima):
            for template_file in ["SERVANT.H", "SERVANT.CPP"]:
                ofile = output_path + '/src/' + iface_name.lower() + 'I.' + template_file.split('.')[-1].lower()
                print('Generating ', ofile, ' (servant for', iface_name + ')')
                # Call cog
                params = {
                    "theCDSL": cdsl_file,
                    "theIDSLs": imports,
                    "theIDSLPaths": '#'.join(include_dirs),
                    "theInterface": iface_name
                }
                cog_command = generate_cog_command(params, "/opt/robocomp/share/robocompdsl/templateCPP/"+template_file, ofile)
                run_cog_and_replace_tags(cog_command, ofile)

    for imp in component['subscribesTo']:
        iface_name = imp
        if not isinstance(iface_name, str):
            iface_name = iface_name[0]
        if communication_is_ice(imp):
            for template_file in ["SERVANT.H", "SERVANT.CPP"]:
                ofile = output_path + '/src/' + iface_name.lower() + 'I.' + template_file.split('.')[-1].lower()
                print('Generating ', ofile, ' (servant for', iface_name + ')')
                # Call cog
                the_interface_str = iface_name
                if isinstance(the_interface_str, list):
                    the_interface_str = str(';'.join(iface_name))
                params = {
                    "theCDSL": cdsl_file,
                    "theIDSLs": imports,
                    "theIDSLPaths": '#'.join(include_dirs),
                    "theInterface": the_interface_str
                }
                cog_command = generate_cog_command(params, "/opt/robocomp/share/robocompdsl/templateCPP/" + template_file, ofile)
                run_cog_and_replace_tags(cog_command, ofile)
    return new_existing_files


def generate_idsl_file(input_file, output_file, include_dirs):
    # idsl = IDSLParsing.fromFileIDSL(inputFile)
    print('Generating ICE file ', output_file)
    # Call cog
    params = {
        "theIDSL": input_file,
        "theIDSLPaths": '#'.join(include_dirs)
    }
    cog_command = generate_cog_command(params, "/opt/robocomp/share/robocompdsl/TEMPLATE.ICE", output_file)
    run_cog_and_replace_tags(cog_command, output_file)


def create_component_directories(output_path, language):
    if not os.path.exists(output_path):
        create_directory(output_path)
    # Create directories within the output directory
    new_dirs = ["bin", "src", "etc"]
    for new_dir in new_dirs:
        if language.lower() == "python" and new_dir == "bin": continue
        try:
            create_directory(os.path.join(output_path, new_dir))
        except:
            print('There was a problem creating a directory %s' % new_dir)
            sys.exit(1)


# Function to create directories
def create_directory(directory):
    try:
        print('Creating', directory,)
        os.mkdir(directory)
        print('')
    except:
        if os.path.isdir(directory):
            print('(already existed)')
            pass
        else:
            raise RuntimeError('\nCOULDN\'T CREATE %s' % directory)


def generate_cog_command(params, template, output_file):
    params_strings = ["-D %s=%s" % (key, value) for key, value in params.items()]
    return "cog.py -z -d %s -o %s %s" % (" ".join(params_strings), output_file, template)


def run_cog_and_replace_tags(cog_command, ofile):
    run = cog_command.split(' ')
    ret = Cog().main(run)
    if ret != 0:
        raise RuntimeError('ERROR (%d) executing cog %s' % (ret, cog_command))
    replaceTagsInFile(ofile)


def main():
    parser = MyParser(description='This application create components files from cdsl files or .ice from idsl\n'
                                  '\ta) to generate code from a CDSL file:     ' + sys.argv[0].split('/')[-1]
                                  + '   INPUT_FILE.CDSL   OUTPUT_PATH\n'
                                  + '\tb) to generate a new CDSL file:           ' + sys.argv[0].split('/')[-1]
                                  + '   NEW_COMPONENT_DESCRIPTOR.CDSL',
                      formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("-I", "--include_dirs", nargs='*', help="Include directories",
                        action=FullPaths, default=[])
    parser.add_argument("-d", '--diff', dest='diff', choices=DIFF_TOOLS, action='store')
    parser.add_argument("input_file", help="The input dsl file")
    parser.add_argument("output_path", nargs='?', help="The path to put the files")
    args = parser.parse_args()

    if args.output_path is None:
        if args.input_file.endswith(".cdsl"):
            generateDummyCDSL(args.input_file)
            generateDummySMDSL("statemachine.smdsl")
            sys.exit(0)
        else:
            print(args.output_path, args.input_file)
            print(parser.error("No output path with non .cdsl file"))
            sys.exit(-1)

    input_file = args.input_file
    output_path = args.output_path

    sys.path.append('/opt/robocomp/python')

    if input_file.endswith(".cdsl"):
        generate_component_from_cdsl(input_file, output_path, args.include_dirs)

    elif input_file.endswith(".idsl"):
        generate_idsl_file(input_file, output_path, args.include_dirs)


if __name__ == '__main__':
    main()
