#!/usr/bin/env python3

# TODO
#
# Read ports from component-ports.txt for the files in etc.
#
#
import argparse
import filecmp
import sys, os, subprocess
from distutils import spawn

from cogapp import Cog
# from parseCDSL import *
sys.path.append("/opt/robocomp/python")
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, gimmeIDSL
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
    idsl = gimmeIDSL(idsl_file, files='', includeDirectories=include_directories)
    if not os.path.exists(output_path):
        create_directory(output_path)

    def generarH(idslFile, imported):
        idsl = DSLFactory().from_file(idslFile)
        try:
            os.system("rm -f " + output_path + "/" + idsl['module']['name'] + "ROS/msg/__init__.py")
            os.system("rm -f " + output_path + "/" + idsl['module']['name'] + "ROS/srv/__init__.py")
        except KeyError:
            print("No module found in %s"%idsl_file)
        for imp in idsl['structs']+idsl['sequences']:
            if imp['type'] in ['struct','sequence']:
                for f in [ "SERVANT.MSG"]:
                    ofile = output_path + "/" + imp['name'] + "." + f.split('.')[-1].lower()
                    print('Generating', ofile, ' (servant for', idslFile.split('.')[0].lower() + ')')

                    ofile_dir = os.path.dirname(ofile)
                    if not os.path.exists(ofile_dir):
                        os.makedirs(ofile_dir)
                    # Call cog
                    run = "cog.py -z -d" + ' -D theIDSLPaths=' + '#'.join(include_directories) + " -D structName=" + imp['name'] + " -D theIDSL=" + idslFile + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
                    run = run.split(' ')
                    ret = Cog().main(run)
                    if ret != 0:
                        print('ERROR')
                        sys.exit(-1)
                    replaceTagsInFile(ofile)
                    commandCPP = "/opt/ros/melodic/lib/gencpp/gen_cpp.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -I" + idsl['name'] + "ROS:" + output_path
                    commandPY  = "/opt/ros/melodic/lib/genpy/genmsg_py.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -I" + idsl['name'] + "ROS:" + output_path
                    for impo in imported:
                        if not impo == idsl['module']['name']+"ROS":
                            commandCPP = commandCPP + " -I" + impo + ":" + output_path
                            commandPY  = commandPY + " -I" + impo + ":" + output_path
                    if not os.path.exists(output_path):
                        create_directory(output_path)
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
                                    for f in [ "SERVANT.SRV"]:
                                        ofile = output_path + "/" + method['name'] + "." + f.split('.')[-1].lower()
                                        print('Generating', ofile, ' (servant for', idslFile.split('.')[0].lower() + ')')
                                        # Call cog
                                        run = "cog.py -z -d" + ' -D theIDSLPaths=' + '#'.join(include_directories) + " -D methodName=" + method['name'] + " -D theIDSL=" + idslFile + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
                                        run = run.split(' ')
                                        ret = Cog().main(run)
                                        if ret != 0:
                                            print('ERROR')
                                            sys.exit(-1)
                                        replaceTagsInFile(ofile)
                                        commandCPP = "/opt/ros/melodic/lib/gencpp/gen_cpp.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -Istd_srvs:/opt/ros/melodic/share/std_srv/cmake/../srv -I" + idsl['module']['name'] + "ROS:" + output_path
                                        commandPY  = "/opt/ros/melodic/lib/genpy/gensrv_py.py " + ofile + " -Istd_msgs:/opt/ros/melodic/share/std_msgs/msg -Istd_srvs:/opt/ros/kinetic/share/std_srv/cmake/../srv -I" + idsl['module']['name'] + "ROS:" + output_path
                                        for impo in imported:
                                            if not impo == idsl['module']['name']+"ROS":
                                                commandCPP = commandCPP + " -I" + impo + ":" + output_path
                                                commandPY  = commandPY + " -I" + impo + ":" + output_path
                                        if not os.path.exists(output_path):
                                            create_directory(output_path)
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
                                    print("error: ROS service with incorrect number of parameters. ROS only supports remote procedure calls of the form: void method(type inVar, out type outVar);")
                                    for param in enumerate(method['params']):
                                        print(param[0], '-->', param[1])
                                    sys.exit(-1)
                            else:
                                print("error: service without params. Form is: void method(type inVar, out type outVar);")
                                sys.exit(-1)
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

#########################################
# Directory structure and other checks  #
#########################################
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
            print('\nCOULDN\'T CREATE', directory)
            sys.exit(-1)

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

#Class to print(colored error message on argparse
class MyParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write((BColors.FAIL + 'error: %s\n' + BColors.ENDC) % message)
        self.print_help()
        sys.exit(2)

#Class to expand and check list of paths arguments in argparse
class FullPaths(argparse.Action):
    """Expand user- and relative-paths"""
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


def main():
    parser = MyParser(description='This application create components files from cdsl files or .ice from idsl\n'
                                  '\ta) to generate code from a CDSL file:     ' + sys.argv[0].split('/')[-1]
                                  + '   INPUT_FILE.CDSL   OUTPUT_PATH\n'
                                  +'\tb) to generate a new CDSL file:           ' + sys.argv[0].split('/')[-1]
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

    inputFile = args.input_file
    outputPath = args.output_path

    sys.path.append('/opt/robocomp/python')

    if inputFile.endswith(".cdsl"):
        generate_component_from_cdsl(inputFile, outputPath, args.include_dirs)

    elif inputFile.endswith(".idsl"):
        generate_idsl_file(inputFile, outputPath, args.include_dirs)

def generate_component_from_cdsl(inputFile, outputPath, include_dirs, diff=False):

    component = DSLFactory().from_file(inputFile, includeDirectories=include_dirs)
    imports = ''.join([imp + '#' for imp in component['imports']])

    # verification
    pool = IDSLPool(imports, include_dirs)
    interface_list = component['requires'] + component['implements'] + component['subscribesTo'] + component[
        'publishes']

    for interface_required in interface_list:
        interface_required = interface_required if isinstance(interface_required, str) else interface_required[0]
        if not pool.moduleProviding(interface_required):
            raise rcExceptions.InterfaceNotFound(interface_required, pool.interfaces())

    if component['language'].lower() == 'cpp' or component['language'].lower() == 'cpp11':
        new_existing_files = generate_cpp_component(component,inputFile,outputPath,include_dirs,imports)
    elif component['language'].lower() == 'python':
        new_existing_files = generate_python_component(component,inputFile,outputPath,include_dirs,imports)
    else:
        print('Unsupported language', component['language'])
        sys.exit(-1)

    if component['usingROS'] == True:
        for imp in component['imports']:
            generate_ROS_headers(imp, outputPath + "/src", component, include_dirs)

    # Code to launch diff tool on .new files to be compared with their old version
    if diff is not None:
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
                    print("Exception trying to execute %s" % (diff_tool))
                    print(e.message)

            else:
                print("Binary equal files %s and %s" % (o_file, n_file))

def generate_python_component(component, inputFile, outputPath, include_dirs, imports):
    #
    # Check output directory
    #
    if not os.path.exists(outputPath):
        create_directory(outputPath)
    # Create directories within the output directory
    try:
        create_directory(outputPath + "/etc")
        create_directory(outputPath + "/src")
    except:
        print('There was a problem creating a directory')
        sys.exit(1)
        pass

    needStorm = False
    for pub in component['publishes']:
        if communication_is_ice(pub):
            needStorm = True
    for sub in component['subscribesTo']:
        if communication_is_ice(sub):
            needStorm = True
    #
    # Generate regular files
    #
    files = ['CMakeLists.txt', 'DoxyFile', 'README-STORM.txt', 'README.md', 'etc/config', 'src/main.py',
             'src/genericworker.py', 'src/specificworker.py', 'src/mainUI.ui']
    specificFiles = ['src/specificworker.py', 'src/mainUI.ui', 'README.md', 'etc/config']

    new_existing_files = {}
    for f in files:
        if f == 'src/main.py':
            ofile = outputPath + '/src/' + component['name'] + '.py'
        else:
            ofile = outputPath + '/' + f
        if f in specificFiles and os.path.exists(ofile):
            print('Not overwriting specific file "' + ofile + '", saving it to ' + ofile + '.new')
            new_existing_files[os.path.abspath(ofile)] = os.path.abspath(ofile) + '.new'
            ofile += '.new'
        ifile = "/opt/robocomp/share/robocompdsl/templatePython/" + f
        ignoreFile = False
        if f == 'src/mainUI.ui' and component['gui'] is None: ignoreFile = True
        if f == 'CMakeLists.txt' and component['gui'] is None: ignoreFile = True
        if f == 'README-STORM.txt' and needStorm == False: ignoreFile = True
        if not ignoreFile:
            print('Generating', ofile)
            run = "cog.py -z -d -D theCDSL=" + inputFile + " -D theIDSLs=" + imports + ' -D theIDSLPaths=' + '#'.join(
                include_dirs) + " -o " + ofile + " " + ifile
            run = run.split(' ')
            ret = Cog().main(run)
            if ret != 0:
                print('ERROR')
                sys.exit(-1)
            replaceTagsInFile(ofile)
            if f == 'src/main.py': os.chmod(ofile, os.stat(ofile).st_mode | 0o111)
    #
    # Generate interface-dependent files
    #
    for imp in component['implements'] + component['subscribesTo']:
        if type(imp) != type(''):
            im = imp[0]
        else:
            im = imp
        if communication_is_ice(imp):
            for f in ["SERVANT.PY"]:
                ofile = outputPath + '/src/' + im.lower() + 'I.' + f.split('.')[-1].lower()
                print('Generating', ofile, ' (servant for', im + ')')
                # Call cog
                run = "cog.py -z -d -D theCDSL=" + inputFile + " -D theIDSLs=" + imports + ' -D theIDSLPaths=' + '#'.join(
                    include_dirs) + " -D theInterface=" + im + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templatePython/" + f
                run = run.split(' ')
                ret = Cog().main(run)
                if ret != 0:
                    print('ERROR')
                    sys.exit(-1)
                replaceTagsInFile(ofile)

def generate_cpp_component(component, inputFile, outputPath, include_dirs, imports):
    #
    # Check output directory
    #
    if not os.path.exists(outputPath):
        create_directory(outputPath)
    # Create directories within the output directory
    try:
        create_directory(outputPath + "/bin")
        create_directory(outputPath + "/etc")
        create_directory(outputPath + "/src")
    except:
        print('There was a problem creating a directory')
        sys.exit(1)
        pass
    #
    # Generate regular files
    #
    files = ['CMakeLists.txt', 'DoxyFile', 'README-STORM.txt', 'README.md', 'etc/config', 'src/main.cpp',
             'src/CMakeLists.txt', 'src/CMakeListsSpecific.txt', 'src/commonbehaviorI.h', 'src/commonbehaviorI.cpp',
             'src/genericmonitor.h', 'src/genericmonitor.cpp', 'src/config.h', 'src/specificmonitor.h',
             'src/specificmonitor.cpp', 'src/genericworker.h', 'src/genericworker.cpp', 'src/specificworker.h',
             'src/specificworker.cpp', 'src/mainUI.ui']
    specificFiles = ['src/specificworker.h', 'src/specificworker.cpp', 'src/CMakeListsSpecific.txt',
                     'src/mainUI.ui', 'src/specificmonitor.h', 'src/specificmonitor.cpp', 'README.md', 'etc/config']
    new_existing_files = {}
    for f in files:
        ofile = outputPath + '/' + f
        if f in specificFiles and os.path.exists(ofile):
            print('Not overwriting specific file "' + ofile + '", saving it to ' + ofile + '.new')
            new_existing_files[os.path.abspath(ofile)] = os.path.abspath(ofile) + '.new'
            ofile += '.new'
        ifile = "/opt/robocomp/share/robocompdsl/templateCPP/" + f
        if f != 'src/mainUI.ui' or component['gui'] is not None:
            print('Generating', ofile)
            run = "cog.py -z -d -D theCDSL=" + inputFile + " -D theIDSLs=" + imports + ' -D theIDSLPaths=' + '#'.join(
                include_dirs) + " -o " + ofile + " " + ifile
            run = run.split(' ')
            ret = Cog().main(run)
            if ret != 0:
                print('ERROR')
                sys.exit(-1)
            replaceTagsInFile(ofile)
    #
    # Generate interface-dependent files
    #
    for ima in component['implements']:
        im = ima
        if type(im) != type(''):
            im = im[0]
        if communication_is_ice(ima):
            for f in ["SERVANT.H", "SERVANT.CPP"]:
                ofile = outputPath + '/src/' + im.lower() + 'I.' + f.split('.')[-1].lower()
                print('Generating ', ofile, ' (servant for', im + ')')
                # Call cog
                run = "cog.py -z -d -D theCDSL=" + inputFile + " -D theIDSLs=" + imports + ' -D theIDSLPaths=' + '#'.join(
                    include_dirs) + " -D theInterface=" + im + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
                run = run.split(' ')
                ret = Cog().main(run)
                if ret != 0:
                    print('ERROR')
                    sys.exit(-1)
                replaceTagsInFile(ofile)

    for imp in component['subscribesTo']:
        im = imp
        if type(im) != type(''):
            im = im[0]
        if communication_is_ice(imp):
            for f in ["SERVANT.H", "SERVANT.CPP"]:
                ofile = outputPath + '/src/' + im.lower() + 'I.' + f.split('.')[-1].lower()
                print('Generating ', ofile, ' (servant for', im + ')')
                # Call cog
                theInterfaceStr = im
                if type(theInterfaceStr) == type([]):
                    theInterfaceStr = str(';'.join(im))
                run = "cog.py -z -d -D theCDSL=" + inputFile + " -D theIDSLs=" + imports + ' -D theIDSLPaths=' + '#'.join(
                    include_dirs) + " -D theInterface=" + theInterfaceStr + " -o " + ofile + " " + "/opt/robocomp/share/robocompdsl/templateCPP/" + f
                # print(run
                run = run.split(' ')
                ret = Cog().main(run)
                if ret != 0:
                    print('ERROR')
                    sys.exit(-1)
                replaceTagsInFile(ofile)
    return new_existing_files

def generate_idsl_file(inputFile, outputPath, include_dirs):
    # idsl = IDSLParsing.fromFileIDSL(inputFile)
    print('Generating ICE file ', outputPath)
    # Call cog
    run = "cog.py -z -d" + " -D theIDSL=" + inputFile + ' -D theIDSLPaths=' + '#'.join(
        include_dirs) + " -o " + outputPath + " /opt/robocomp/share/robocompdsl/TEMPLATE.ICE"
    run = run.split(' ')
    ret = Cog().main(run)
    if ret != 0:
        print('ERROR')
        sys.exit(-1)
    replaceTagsInFile(outputPath)

if __name__ == '__main__':
    app = main()
