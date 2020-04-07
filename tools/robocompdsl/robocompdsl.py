#!/usr/bin/env python3

# TODO
#
# Read ports from component-ports.txt for the files in etc.
#
#
import argparse
import os
import sys

from termcolor import colored

sys.path.append("/opt/robocomp/python")
import rcExceptions
from componentgenerator import ComponentGenerator
import robocompdslutils


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


class MyArgsParser(argparse.ArgumentParser):
    """
    Class to print(colored error message on argparse
    """
    def error(self, message):
        sys.stderr.write(colored('error: %s\n' % message, 'red'))
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


def generate_idsl_file(input_file, output_file, include_dirs):
    # idsl = IDSLParsing.fromFileIDSL(inputFile)
    print('Generating ICE file ', output_file)
    # Call cog
    params = {
        "theIDSL": input_file,
        "theIDSLPaths": '#'.join(include_dirs)
    }
    cog_command = robocompdslutils.generate_cog_command(params, "/opt/robocomp/share/robocompdsl/TEMPLATE.ICE", output_file)
    robocompdslutils.run_cog_and_replace_tags(cog_command, output_file)


def main():
    parser = MyArgsParser(description='This application create components files from cdsl files or .ice from idsl\n'
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

    if input_file.endswith(".cdsl") or input_file.endswith(".jcdsl"):
        ComponentGenerator().generate(input_file, output_path, args.include_dirs, args.diff)

    elif input_file.endswith(".idsl"):
        generate_idsl_file(input_file, output_path, args.include_dirs)
        
    else:
        print("Please check the Input file \n" + "Input File should be either .cdsl or .idsl")
        sys.exit(-1)

if __name__ == '__main__':
    main()
