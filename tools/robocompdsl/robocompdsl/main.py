#!/usr/bin/env python3

# TODO
#
# Read ports from component-ports.txt for the files in etc.
#
#
from typing import Optional, List
from pathlib import Path
import typer
import os
import sys
import pyparsing
import rich
from rich.console import Console

from common.filesgenerator import FilesGenerator

DESCRIPTION_STR = """\
This application create components files from cdsl files or .ice from idsl
    a) to generate code from a CDSL file:\t{name}    INPUT_FILE.CDSL    OUTPUT_PATH
    b) to generate a new CDSL file:\t\t{name}    NEW_COMPONENT_DESCRIPTOR.CDSL
    c) to generate .ice from a IDSL file:\t{name}    INPUT_FILE.idsl    OUTPUT_FILE_PATH.ice
"""

app = typer.Typer(help=DESCRIPTION_STR)

console = Console()

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


@app.command()
def generate(
        input_file: str = typer.Argument(..., help="The input dsl file"),
        output_path: Optional[str] = typer.Argument(None, help="The path to put the generated files"),
        include_dirs: List[Path] = typer.Option([],  "--include_dirs", "-I", help="List of directories to find includes."),
        diff: bool = typer.Option(False, "--diff", "-d", help="Show the diff of the old and new files"),
        test: bool = typer.Option(False, "--test", "-t",  help="Testing option")
):
    if output_path is None:
        if input_file.endswith(".cdsl"):
            generate_dummy_CDSL(input_file)
            generate_dummy_SMDSL("statemachine.smdsl")
            sys.exit(0)
        else:
            print(output_path, input_file)
            print(parser.error("No output path with non .cdsl file"))
            sys.exit(-1)
    for i_dir in include_dirs:
        if not i_dir.is_dir():
            console.log(f"{i_dir} directory in -I option  not exists")
            return -1
    if input_file.endswith(".cdsl") or input_file.endswith(".jcdsl") or input_file.endswith(".idsl"):
        try:
            # print("To generate", input_file, output_path, include_dirs, diff, test)
            FilesGenerator().generate(input_file, output_path, include_dirs, diff, test)
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
    app()
