#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
from __future__ import print_function
import argparse, argcomplete
import os
import string
import sys
from typing import Optional, List

from rich import console
import typer
sys.path.append('/opt/robocomp/python')
from rcworkspace.workspace import Workspace
try:
    from pyaku.pyaku import Yaku
except ModuleNotFoundError:
    Yaku = False

app = typer.Typer(help=typer.style("Shortcut tool to run Robocomp components.", fg=typer.colors.GREEN))
console = console.Console()

class rcrun:
    def __init__(self, workspace=None):
        if not workspace:
            workspace = Workspace()
        self.ws = workspace

    def complete_components(self, prefix, parsed_args, **kwargs):
        components = self.ws.list_filtered_components_names(prefix)
        componentsname=[]
        for component in components:
            componentsname.append(component.split('/')[ len(component.split('/')) -1 ])
        return (componentname for componentname in componentsname if componentname.startswith(prefix))

    def complete_scripts(self, prefix, parsed_args, **kwargs):
        configPath = self.ws.find_component_exec('testcomp1')
        if not configPath: return
        configPath = configPath[:-4]+ '/etc'
        configFiles = []
        for root, dirs, files in os.walk(configPath): configFiles = files
        return (configfile for configfile in configFiles if configfile.startswith(prefix))

    def find_script(self, action,component):
        paths = self.ws.find_component_exec(component)
        pathsrc = self.ws.find_component_src(component)
        pathsrc = [ os.path.join(x,'bin') for x in pathsrc]
        pathsrc.append(paths)
        for path in pathsrc:
            for file in os.listdir(path):
                if file[-3:] == '.sh' or file[-5:] == '.bash':
                    if string.lower(file[:len(action)])==string.lower(action):
                        return os.path.join(path,file)
        return False

def main():
    parser = argparse.ArgumentParser(description="Locate and run a robocomp component")
    group = parser.add_mutually_exclusive_group(required=False)
    cgroup = parser.add_mutually_exclusive_group()
    ws = Workspace()
    rcrun_instance = rcrun(ws)
    parser.add_argument('component', help='component name').completer = rcrun_instance.complete_components
    group.add_argument('-s', '--start', action = 'store_true' , help="start component")
    group.add_argument('-st', '--stop', action = 'store_true' , help="stop component")
    group.add_argument('-fst', '--fstop', action = 'store_true' , help=" force stop component")
    cgroup.add_argument('-d', '--debug', action = 'store_true' , help="start a component in debug mode")
    cgroup.add_argument('-cf', '--cfile', nargs = 1 , help="use custom ice config file (absolute path)")
    cgroup.add_argument('-c', '--config', nargs = 1 , help="ice config file to choose from default config directory").completer = rcrun_instance.complete_scripts
    parser.add_argument('-is','--ignore_scripts', action='store_true',help="ignore all the script files if found")
    
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    component = args.component

    if not args.start and not args.stop and not args.fstop:
        args.start = True

    #if stop command no need for searching all
    if args.stop:
        if not ws.find_component_exec(component):
            args.error("couldnt find the component %s in any of the workspaces" % (args.stop[0]))
            return
        stpath = rcrun_instance.find_script("stop", component)
        if stpath and not args.ignore_scripts:
            #print("using script {0}".format(stpath))
            command = stpath
        else:
            command = "killall " + str(component)
        os.system(command)
        return
    elif args.fstop:
        if not ws.find_component_exec(component):
            parser.error("couldnt find the component %s in any of the workspaces" % (args.fstop[0]))
            return
        sfpath = rcrun_instance.find_script("forcestop",component)
        if sfpath and not args.ignore_scripts:
            #print("using script {0}".format(sfpath))
            command = sfpath
        else:
            command = "killall -9 " + str(component)
        os.system(command)
        return
    
    
    #search for the component
    component_path = ws.find_component(component)
    component = component_path.rstrip(os.path.sep).split(os.path.sep)[-1]
    if not component_path:
        print("Couldn't find the component path for %s in any of the workspaces" % (component))
        return
    component_exec_path = ws.find_component_exec_file(component_path)
    component_etc_path = ws.find_component_etc_path(component_path)

    


    #find the config file
    '''
        if only one file is present in the etc directory then it will be used
        else if it has file named config then it will be used 
        if it has file named generic_config it will be used
        else we will use a random file
        user defined config file will override all the above
    '''
    config_files = []
    ice_config = ""
    for root, dirs, config_files in os.walk(component_etc_path):
        if len(config_files) == 0:
            parser.error("couldnt find any config file in the etc directory, please specify a custom config file\n")
        else:
            if args.debug:
                for file in config_files:
                    if file.endswith('.debug'):
                        ice_config = os.path.join(component_etc_path, file)
                if len(ice_config) < 9:
                    parser.error("Couldn't find any debug config file in the etc directory, please specify a custom config file\n")
            else:
                if len(config_files) == 1:
                    ice_config = os.path.join(component_etc_path, config_files[0])
                elif len(config_files) > 1:
                    if "config" in config_files:
                        ice_config = os.path.join(component_etc_path, "config")
                    elif 'generic_config' in  config_files:
                        ice_config = os.path.join(component_etc_path, "config")
                    else:
                        ice_config = os.path.join(component_etc_path, config_files[0])
                else:
                    parser.error(
                        "Couldn't find any config file in the etc directory, please specify a custom config file\n")
    
    if args.config:
        if args.config[0] in config_files:
            ice_config = os.path.join(component_etc_path, args.config[0])
        else:
            parser.error("couldnt find config file '{0}' in the etc directory".format(args.config[0]))
    elif args.cfile:
        ice_config = args.config[0]

    #execute the command
    # spath = rcrun_instance.find_script("start",component);
    # sdpath = rcrun_instance.find_script("startdebug",component);
    
    if args.start:
        command = "cd "+component_path+" && "
        command += component_exec_path + " " + ice_config
        if component_exec_path.endswith('.py'):
            command = "python "+command


    
    print("executing : "+command)
    if Yaku:
        Yaku().rename_current_tab(name=component)
    os.system(command)


ws = Workspace()
runner = rcrun()


@app.command()
def start(
        component_names: Optional[List[str]] = typer.Argument(None, help="Lost of component names")
):
    """
    start component
    """
    for component_name in component_names:
        component = ws.find_component(component_name)
        component.run_component(use_yaku=True)


@app.command()
def stop(
        component_name: str = typer.Argument("", help="Component name"),
        ignore_scripts: bool = typer.Option(False, help="Ignore existing scripts to run")
):
    """
    stop component
    """
    if not ws.find_component_exec(component_name):
        console.log(f"couldn't find the component {component_name} in any of the workspaces")
        return
    script_path = runner.find_script("stop", component_name)
    if script_path and not ignore_scripts:
        # print("using script {0}".format(script_path))
        command = script_path
    else:
        command = "killall " + str(component_name)
    os.system(command)
    return


@app.command()
def fstop(
        component_name: str = typer.Argument("", help="Component name"),
        ignore_scripts: bool = typer.Option(False, help="Ignore existing scripts to run")
):
    """
    force stop component
    """
    if not ws.find_component_exec(component_name):
        console.log(f"couldn't find the component {component_name} in any of the workspaces")
        return
    script_path = runner.find_script("forcestop", component_name)
    if script_path and not ignore_scripts:
        # print("using script {0}".format(script_path))
        command = script_path
    else:
        command = "killall -9 " + str(component_name)
    os.system(command)
    return

@app.command()
def debug(
        component_name: str = typer.Argument("", help="Component name")
):
    """
    start a component in debug mode
    """

# cgroup.add_argument('-cf', '--cfile', nargs = 1 , help="use custom ice config file (absolute path)")
# cgroup.add_argument('-c', '--config', nargs = 1 , help="ice config file to choose from default config directory").completer = rcrun_instance.complete_scripts
# parser.add_argument('-is','--ignore_scripts', action='store_true',help="ignore all the script files if found")

if __name__ == '__main__':
    app()


