#!/usr/bin/env python3

# TODO
#
# Read ports from component-ports.txt for the files in etc.
#
#
import argparse
import os
import sys
import pyparsing
import rich
from rich.text import Text


sys.path.append("/opt/robocomp/python")
sys.path.append('/opt/robocomp/share/robocompdsl/')
import rcExceptions
from filesgenerator import FilesGenerator
import robocompdslutils


console = rich.console.Console()

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
    //options dsr, agmagent, InnerModelViewer;
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


def generate_dummy_CDSL(path):
    if os.path.exists(path):
        console.print(f"File {path} already exists.\nNot overwritting.", style='yellow')
    else:
        console.print(f"Generating dummy CDSL file: {path}")

        name = path.split('/')[-1].split('.')[0]
        string = DUMMY_CDSL_STRING.replace('<CHANGETHECOMPONENTNAME>', name)
        open(path, "w").write(string)


def generate_dummy_SMDSL(path):
    if os.path.exists(path):
        console.print(f"File {path} already exists.\nNot overwritting.", style='yellow')
    else:
        console.print(f"Generating dummy SMDSL file: {path}", style='green')

        open(path, "w").write(DUMMY_SMDSL_STRING)


class MyArgsParser(argparse.ArgumentParser):
    """
    Class to print colored error message on argparse
    """
    def error(self, message):
        console.log(Text('error: %s\n' % message, style='red'))
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



DESCRIPTION_STR = """\
This application create components files from cdsl files or .ice from idsl
    a) to generate code from a CDSL file:\t{name}    INPUT_FILE.CDSL    OUTPUT_PATH
    b) to generate a new CDSL file:\t\t{name}    NEW_COMPONENT_DESCRIPTOR.CDSL
    c) to generate .ice from a IDSL file:\t{name}    INPUT_FILE.idsl    OUTPUT_FILE_PATH.ice
"""


def main():
    parser = MyArgsParser(prog='robcompdsl',
                          description=DESCRIPTION_STR.format(name=sys.argv[0].split('/')[-1]),
                          formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-I", "--include_dirs", nargs='*', help="Include directories",
                        action=FullPaths, default=[])
    parser.add_argument("-d", '--diff', dest='diff', choices=DIFF_TOOLS, action='store')
    parser.add_argument("-t", '--test', dest='test', action='store_true')
    parser.add_argument("input_file", help="The input dsl file")
    parser.add_argument("output_path", nargs='?', help="The path to put the generated files")
    args = parser.parse_args()

    if args.output_path is None:
        if args.input_file.endswith(".cdsl"):
            generate_dummy_CDSL(args.input_file)
            generate_dummy_SMDSL("statemachine.smdsl")
            sys.exit(0)
        else:
            print(args.output_path, args.input_file)
            print(parser.error("No output path with non .cdsl file"))
            sys.exit(-1)

    input_file = args.input_file
    output_path = args.output_path

    if input_file.endswith(".cdsl") or input_file.endswith(".jcdsl") or input_file.endswith(".idsl"):
        try:
            FilesGenerator().generate(input_file, output_path, args.include_dirs, args.diff, args.test)
        except pyparsing.ParseException as pe:
            console.log(f"Error generating files for {rich.Text(input_file, style='red')}")
            console.log(pe.line)
            console.log(' ' * (pe.col - 1) + '^')
            console.log(pe)
            exit(-1)

    else:
        console.print("Please check the Input file \n" + "Input File should be either .cdsl or .idsl")
        sys.exit(-1)


if __name__ == '__main__':
    main()
