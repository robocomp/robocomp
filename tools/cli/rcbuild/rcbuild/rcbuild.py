#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
from __future__ import print_function

import multiprocessing
import os
import shlex
import subprocess
from pathlib import Path
from typing import Optional
import typer

from robocomp import execute_command, is_interactive
from rcworkspace.workspace import Workspace
from rcconfig.rcconfig import RC_CONFIG
import rcdocker.rcdocker

app = typer.Typer(help=typer.style("Robocomp command to build components or robocomp itself. There are also several "
                                   "options to clean, rebuild or install.\n"
                                   "The name of the component to be used doesn't need to be a full path. The passed name"
                                   "will be looked for inside the defined workspaces.", fg=typer.colors.GREEN))

# TODO: Move to robocomp config py file
ROBOCOMP_INSTALL_DIR = "/opt/robocomp"
BUILD_DIR = Path(RC_CONFIG["ROBOCOMP_SRC"]) / "build"

class RCBuild:
    def __init__(self):
        self.ws = Workspace()

    def build_component(self, bcomponent, do_clean_first):
        if component := self.ws.find_component(bcomponent, is_interactive()):
            component.build()

    def remove_cmakecache_files(self, path: Path):
        if 'build' in path:
            new_path = path
            while True:
                last_dir = os.path.basename(os.path.normpath(new_path))
                len_to_remove = len(last_dir)+1 if new_path.endswith('/') else len(last_dir)
                new_path = new_path[:-len_to_remove]
                if 'build' == last_dir and not new_path.strip() == "":
                    new_path = os.path.join(new_path, "../../build")
                    break
        elif os.path.isdir(current := os.path.join(path, "../../build")):
                new_path = current
        else:
            new_path = self.ws.find_component(path)

        if new_path is not None:
            if os.path.exists(cmake_file_path := os.path.join(new_path, "CMakeCache.txt")):
                os.remove(cmake_file_path)
            if os.path.exists(cmake_file_path := os.path.join(new_path, "../../..", "CMakeCache.txt")):
                os.remove(cmake_file_path)
            return True
        return False

    def remove_undesired_files(self, path: Path) -> bool:
        return self.remove_cmakecache_files(path)

    def build_docs(self, component, install=False, installpath='/opt/robocomp'):
        path = self.ws.find_component(component)
        if not path:
            print("No such component exists")
            return False
        os.chdir(path)
        if install:
            try:
                os.system('mkdir -p '+os.path.join(installpath, 'doc'))
                os.system('sudo cp -R doc/html '+installpath+'/doc/'+component.lower())
            except Exception as e:
                raise RuntimeError("Couldn't install doc files {0}".format(e))
        else:
            try:
                os.system('doxygen Doxyfile')
            except Exception as e:
                raise RuntimeError("Couldn't generate doc files {0}".format(e))

builder = RCBuild()


@app.command(name="doc", help=typer.style("Build documentation of the specified component", fg=typer.colors.GREEN))
def build_docs(
        component: Optional[str] = typer.Argument(None, help='Name of the component to build, if omitted current dir is used.'),
        install: Optional[bool] = typer.Option(False, '--clean', '-c', help="install documentation")
):
    if not component or component.strip() == '.':
        component_path = os.getcwd()
    else:
        component_path = component
    builder.build_docs(component_path, install)


@app.command(name="comp",  help=typer.style("Build the specified component.", fg=typer.colors.GREEN))
def build_component(
    component: Optional[str] = typer.Argument(None, help='Name of the component to build, if omitted current dir is used.'),
    do_clean: Optional[bool] = typer.Option(False, '--clean', help="Clean the compilation files (CMake and Make generated files."),
    isolated: Optional[bool] = typer.Option(False, '--isolated', help="Build component in docker image")
):
    if not component or component.strip() == '.':
        component_path = os.getcwd()
    else:
        component_path = component
    if isolated:
            rcdocker.rcdocker.build_component_in_container(component)
    else:
        builder.build_component(component_path, do_clean)


@app.command(name="clean",
             help=typer.style("Clean the CMake and Make temp files of the specified component.", fg=typer.colors.GREEN),
             short_help=typer.style("Clean CMake dir and Make files of the component.", fg=typer.colors.GREEN)
             )
def clean(
        component: Optional[str] = typer.Argument(None,
                                                  help='Name of the component to clean, if omitted current dir is used.'),
):
    if not component or component.strip() == '.':
        component_path = os.getcwd()
        typer.secho(f"Using current path as component ({component_path})", fg=typer.colors.YELLOW)
    if not builder.remove_undesired_files(component_path):
        print("Nothing to be removed")


@app.command(help=typer.style("Build robocomp itself.", fg=typer.colors.GREEN))
def robocomp(
        clear_installation: bool = typer.Option(False, "--clear-installation", help="Use this to remove robocomp install dir (/opt/robocomp) prior to build."),
        clear_build: bool = typer.Option(False, "-c", "--clear-build", help="Use this to remove robocomp build dir (<ROBOCOMP_DIR>/build/)"),
        install: bool = typer.Option(False, "-i", "--install", help="Install robocomp after building."),
        dsr: bool = typer.Option(True, "--DSR", help="Compile with DSR support."),
        fcl: bool = typer.Option(True, "--FCL", help="Compile with FCL support."),

):
    # sudo rm -r /opt/robocomp
    if clear_installation:
        if os.path.isdir(ROBOCOMP_INSTALL_DIR):
            command = f"/usr/bin/sudo rm -r {ROBOCOMP_INSTALL_DIR}"
            execute_command(command)

    if clear_build and BUILD_DIR.exists():
        # make clean;
        os.chdir(BUILD_DIR)
        if (BUILD_DIR / "Makefile").exists():
            execute_command("make clean")
        if (BUILD_DIR.parent / "CMakeCache.txt").exists():
            execute_command(f"rm {BUILD_DIR.parent / 'CMakeCache.txt'}")
        # cd ..;
        os.chdir(RC_CONFIG["ROBOCOMP_SRC"])
        # sudo rm -r build;
        execute_command(f"/usr/bin/sudo rm -r {BUILD_DIR}")

    os.chdir(RC_CONFIG["ROBOCOMP_SRC"])
    if not BUILD_DIR.exists():
        # mkdir build;
        execute_command(f"mkdir -p {BUILD_DIR}")

    # cd build;
    os.chdir(BUILD_DIR)
    # cmake -DDSR=TRUE -DFCL_SUPPORT=TRUE .. ;
    execute_command(f"cmake -DDSR={str(dsr).upper()} -DFCL_SUPPORT={str(fcl).upper()} ..")
    # make -j10
    number_of_processors = int(multiprocessing.cpu_count()/2)
    print(f"Compiling with {number_of_processors} processors")
    execute_command(f"make -j{number_of_processors}")
    if install:
        # sudo make install
        execute_command("/usr/bin/sudo make install")
    return True

if __name__ == '__main__':
    app()
