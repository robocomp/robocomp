#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
from __future__ import print_function
import argparse, argcomplete
import multiprocessing
import os
import subprocess
import sys
from typing import Optional

import typer
from prompt_toolkit.shortcuts import confirm
from termcolor import colored

sys.path.append('/opt/robocomp/python')

from workspace import Workspace

app = typer.Typer()

# TODO: Move to robocomp config py file
ROBOCOMP_INSTALL_DIR = "/opt/robocomp"
REPO_DIR = "/home/robolab/robocomp/"
BUILD_DIR = os.path.join(REPO_DIR, "build")

class RCBuild:
    def __init__(self):
        self.ws = Workspace()

    def build_component(self, bcomponent, do_clean_first):
        path = self.ws.find_component(bcomponent)
        if not path:
            print(f"No such {bcomponent} component exists")
            return False

        os.chdir(path)
        build_path = os.path.join(path, 'build')
        if not os.path.exists(build_path):
            os.mkdir(build_path)
        self.remove_undesired_files(path)
        os.chdir(build_path)
        if do_clean_first:
            if os.path.exists('Makefile'):
                os.system("make clean")
            if os.path.exists('CMakeCache.txt'):
                os.remove(os.path.join(build_path, 'CMakeCache.txt'))
        print(f"Working on dir {os.getcwd()}")
        os.system('cmake ..')
        os.system('make')

    def remove_cmake_cache_files(self, path):
        if 'build' in path:
            new_path = path
            while True:
                last_dir = os.path.basename(os.path.normpath(new_path))
                new_path = new_path[:-len(last_dir)]
                if 'build' in last_dir or not new_path.strip() == "":
                    new_path = os.path.join(new_path, "build")
                    break
        elif os.path.isdir(current := os.path.join(path, "build")):
                new_path = current
        else:
            new_path = self.ws.find_component(path)

        if new_path is not None:
            if os.path.exists(cmake_file_path := os.path.join(new_path, "CMakeCache.txt")):
                os.remove(cmake_file_path)
            if os.path.exists(cmake_file_path := os.path.join(new_path, "..", "CMakeCache.txt")):
                os.remove(cmake_file_path)
            return True
        return False

    def remove_undesired_files(self, path) -> bool:
        return self.remove_cmake_cache_files(path)

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


@app.command(name="doc")
def build_docs(
        component: Optional[str] = typer.Argument(None, help='Name of the component to build, if omitted current dir is used.'),
        install: Optional[bool] = typer.Option(False, '--clean', '-c', help="install documentation")
):
    if not component or component.strip() == '.':
        component_path = os.getcwd()
    else:
        component_path = component
    builder.build_docs(component_path, install)


@app.command(name="comp")
def build_components(
    component: Optional[str] = typer.Argument(None, help='Name of the component to build, if omitted current dir is used.'),
    do_clean: Optional[bool] = typer.Option(False, '--install', help="Clean the compilation files (CMake and Make generated files.")
):
    if not component or component.strip() == '.':
        component_path = os.getcwd()
    else:
        component_path = component
    builder.build_component(component_path, do_clean)


@app.command(name="clean")
def clean(
        component: Optional[str] = typer.Argument(None,
                                                  help='Name of the component to build, if omitted current dir is used.'),
):
    if not component or component.strip() == '.':
        component_path = os.getcwd()
    if not builder.remove_undesired_files(component_path):
        print("Nothing to be removed")


@app.command()
def robocomp(
        clear_installation: bool = typer.Option(False, "--clear-installation", help="Use this to remove robocomp install dir (/opt/robocomp)"),
        clear_build: bool = typer.Option(False, "-c", "--clear-build", help="Use this to remove robocomp build dir (/opt/robocomp)"),
        install: bool = typer.Option(False, "-i", "--install", help="Install robocomp."),
        dsr: bool = typer.Option(True, "--DSR", help="Compile with DSR."),
        fcl: bool = typer.Option(True, "--FCL", help="Compile with FCL."),

):
    # sudo rm -r /opt/robocomp
    if clear_installation:
        if os.path.isdir(ROBOCOMP_INSTALL_DIR):
            returncode = subprocess.call(["/usr/bin/sudo", "rm", "-r", f"{ROBOCOMP_INSTALL_DIR}"])

    if clear_build:
        # make clean;
        os.chdir(BUILD_DIR)
        if os.path.isfile("Makefile"):
            subprocess.call(["make", "clean"])
        # TODO: remove cmake cache from build and parent dir
        # cd ..;
        os.chdir(REPO_DIR)
        # sudo rm -r build;
        returncode = subprocess.call(["/usr/bin/sudo", "rm", "-r", "build"])
        # mkdir build;
        returncode = subprocess.call(["mkdir", "-p", BUILD_DIR])
    # cd build;
    # TODO: Replace by robocomp source code dir
    os.chdir(BUILD_DIR)
    # cmake -DDSR=TRUE -DFCL_SUPPORT=TRUE .. ;
    returncode = subprocess.call(["cmake", f"-DDSR={str(dsr).upper()}", f"-DFCL_SUPPORT={str(fcl).upper()}", ".."])
    # make -j10
    number_of_processors = int(multiprocessing.cpu_count()/2)
    print(f"Compiling with {number_of_processors} processors")
    returncode = subprocess.call(["make", f"-j{number_of_processors}"])
    if install:
        # sudo make install
        returncode = subprocess.call(["/usr/bin/sudo", "make", "install"])
    return True

if __name__ == '__main__':
    app()
