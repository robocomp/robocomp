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
from rcworkspace.component_dir import ComponentDir
from rcconfig import RC_CONFIG

app = typer.Typer(short_help=typer.style("Command to build components or robocomp itself", fg=typer.colors.GREEN), help=typer.style("Robocomp command to build components or robocomp itself. There are also several "
                                   "options to clean, rebuild or install.\n"
                                   "The name of the component to be used doesn't need to be a full path. The passed name "
                                   "will be looked for inside the defined workspaces.", fg=typer.colors.GREEN))



class RCBuild:
    def __init__(self):
        self.ws = Workspace()

    def build_component(self, bcomponent, do_clean_first=False, reg_exp=False, all_comps=False):
        component = self.ws.find_component(bcomponent, is_interactive(), reg_exp=reg_exp)
        if all_comps:
            components = self.ws.find_components(bcomponent, reg_exp=reg_exp)
            for component in components:
                component.build(clean=do_clean_first)
        elif component:
            component.build(clean=do_clean_first)
        else:
            component = ComponentDir.create_component(Path(bcomponent))
            if component is not None:
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

    def remove_undesired_files(self, comp_path: str, recursively: bool = False, interactive=False) -> bool:
        if recursively:
            files_removed = []
            comp_path = Path(comp_path).resolve()
            for file_path in comp_path.rglob('*/CMakeCache.txt'):
                print(file_path)
        elif component := self.ws.find_component(comp_path, is_interactive()):
            return component.clean_build()
        return False

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
    component: Optional[Path] = typer.Argument(Path.cwd(), help='Name of the component to build, if omitted current dir is used.'),
    do_clean: Optional[bool] = typer.Option(False, '--clean', help="Clean the compilation files (CMake and Make generated files."),
    isolated: Optional[bool] = typer.Option(False, '--isolated', help="Build component in docker image"),
    reg_exp: Optional[bool] = typer.Option(False, '-r', '--reg-exp', help="Use regular expression to find components"),
    all_comps: Optional[bool] = typer.Option(False, '-a', '--all', help="build all components matching name")
):
    if isolated:
        # import locally to avoid to launch docker client if not needed
        import rcdocker.rcdocker
        rcdocker.rcdocker.build_component_in_container(component)
    else:
        builder.build_component(component, do_clean, reg_exp, all_comps)


@app.command(name="clean",
             help=typer.style("Clean the CMake and Make temp files of the specified component.", fg=typer.colors.GREEN),
             short_help=typer.style("Clean CMake dir and Make files of the component.", fg=typer.colors.GREEN)
             )
def clean(
        component: Optional[str] = typer.Argument(None,
                                                  help='Name of the component to clean, if omitted current dir is used.'),
        recursively: Optional[bool] = typer.Option(False, "-r",
                                                  help='recursively remove CMakeCache.txt files'),
):
    if not component or component.strip() == '.':
        component_path = os.getcwd()
    else:
        component_path = component
    if not builder.remove_undesired_files(component_path, recursively, is_interactive()):
        print("Nothing to be removed")


@app.command(help=typer.style("Build robocomp itself.", fg=typer.colors.GREEN))
def robocomp(
        clear_installation: bool = typer.Option(False, "--clear-installation", help="Use this to remove robocomp install dir (/opt/robocomp) prior to build."),
        clear_build: bool = typer.Option(False, "-c", "--clear-build", help="Use this to remove robocomp build dir (<ROBOCOMP_DIR>/build/)"),
        install: bool = typer.Option(False, "-i", "--install", help="Install robocomp after building."),
        dsr: bool = typer.Option(False, "--DSR", help="Compile with DSR support."),
        fcl: bool = typer.Option(False, "--FCL", help="Compile with FCL support."),
        rcis: bool = typer.Option(False, "--RCIS", help="Compile RCIS tool."),
):
    if clear_installation:
        if (RC_CONFIG.ROBOCOMP_BUILD_DIR / "Makefile").exists():
            os.chdir(RC_CONFIG.ROBOCOMP_BUILD_DIR)
            execute_command("/usr/bin/sudo make uninstall")
    if clear_build and RC_CONFIG.ROBOCOMP_BUILD_DIR.exists():
        # make clean;
        os.chdir(RC_CONFIG.ROBOCOMP_BUILD_DIR)
        if (RC_CONFIG.ROBOCOMP_BUILD_DIR / "Makefile").exists():
            execute_command("make clean")
        if (RC_CONFIG.ROBOCOMP_BUILD_DIR.parent / "CMakeCache.txt").exists():
            execute_command(f"rm {RC_CONFIG.ROBOCOMP_BUILD_DIR.parent / 'CMakeCache.txt'}")
        # cd ..;
        os.chdir(RC_CONFIG.ROBOCOMP_SRC_DIR)
        # sudo rm -r build;
        execute_command(f"/usr/bin/sudo rm -r {RC_CONFIG.ROBOCOMP_BUILD_DIR}")

    if not RC_CONFIG.ROBOCOMP_BUILD_DIR.exists():
        # mkdir build;
        execute_command(f"mkdir -p {RC_CONFIG.ROBOCOMP_BUILD_DIR}")

    # install robocomp cli tools
    execute_command(f"sudo pip install {RC_CONFIG.ROBOCOMP_SRC_DIR/'tools'/'cli'}")
    # cd build;
    os.chdir(RC_CONFIG.ROBOCOMP_BUILD_DIR)
    # cmake -DDSR=TRUE -DFCL_SUPPORT=TRUE .. ;
    if dsr:
        fcl = True
    execute_command(f"cmake -DDSR={str(dsr).upper()} -DFCL_SUPPORT={str(fcl).upper()} -DRCIS={str(rcis).upper()} ..")
    # make -j10
    number_of_processors = int(multiprocessing.cpu_count()/2)
    print(f"Compiling with {number_of_processors} processors")
    execute_command(f"make -j{number_of_processors}")

    # sudo rm -r /opt/robocomp
    if clear_installation:
        commands = ["sudo pip uninstall -y robocompcli", "pip uninstall -y robocompcli"]
        for command in commands:
            execute_command(command)

        os.chdir(RC_CONFIG.ROBOCOMP_SRC_DIR)
        if os.path.isdir(RC_CONFIG.ROBOCOMP_INSTALL_DIR):
            command = f"/usr/bin/sudo rm -r {RC_CONFIG.ROBOCOMP_INSTALL_DIR}"
            execute_command(command)
        # TODO: get the files from other places, not hardcoded
        files_to_remove = [Path("/opt/robocomp/"), Path("/usr/bin/yaku")]
        for installed_file in files_to_remove:
            if installed_file.exists():
                execute_command(f"/usr/bin/sudo rm -v -r {installed_file}")

    if install:
        # sudo make install
        execute_command("/usr/bin/sudo make install")
    return True

if __name__ == '__main__':
    try:
        app()
    except FileNotFoundError as e:
        print("You probably are in a non existing directory.")
