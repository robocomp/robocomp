#!/usr/bin/python
# PYTHON_ARGCOMPLETE_OK
from __future__ import print_function
import argparse, argcomplete
import os
import sys

from termcolor import colored

sys.path.append('/opt/robocomp/python')

from workspace import Workspace


def complete_components(self, prefix, parsed_args, **kwargs):
    components = self.ws.list_components_in_workspace(self.ws.workspace_paths)
    componentsname=[]
    for component in components:
        componentsname.append(component.split('/')[ len(component.split('/')) -1 ])
    return (componentname for componentname in componentsname if componentname.startswith(prefix))

class RCBuild:
    def __init__(self):
        self.ws = Workspace()



    def build_component(self, bcomponent, install):
        clean = False
        path = self.ws.find_component(bcomponent)
        if not path:
            print(f"No such {bcomponent} component exists")
            return False

        #build
        os.chdir(path)
        build_path = os.path.join(path, 'build')
        if not os.path.exists(build_path):
            os.mkdir(build_path)
        self._remove_undesired_files(path)
        os.chdir(build_path)
        if clean:
            if os.path.exists('Makefile'):
                os.system("make clean")
            if os.path.exists('CMakeCache.txt'):
                os.remove(os.path.join(build_path, 'CMakeCache.txt'))
        print(f"Working on dir {os.getcwd()}")
        os.system('cmake ..')
        os.system('make')

    def _remove_undesired_files(self, path):
        if 'build' not in path:
            if 'build' in os.listdir(path):
                #CMakeCache out of build directory.
                if os.path.exists(os.path.join(path,"CMakeCache.txt")):
                    os.remove(os.path.join(path,"CMakeCache.txt"))



    def build_docs(self, component, install=False,installpath='/opt/robocomp'):
        paths = self.ws.find_component_src(component)
        if not paths:
            print("No such component exists")
            return False
        path = paths[0]
        os.chdir(path)
        if install == True:
            try:
                os.system('mkdir -p '+installpath+'/doc')
                os.system(' sudo cp -R doc/html '+installpath+'/doc/'+component.lower())
            except Exception as e:
                raise RuntimeError("couldnt install doc files {0}".format(e))
        else:
            try:
                os.system('doxygen Doxyfile')
            except Exception as e:
                raise RuntimeError("couldnt generate doc files {0}".format(e))

def main():
    parser = argparse.ArgumentParser(description="configures and build components ")
    group = parser.add_mutually_exclusive_group()
    parser.add_argument('component', nargs='?', help='name of the component to build, if omitted curent workspace is build').completer = complete_components
    group.add_argument('-i','--install',nargs='?' , default = 'notgiven' , help="install the component(s) to given path(relative from build space or abs), defaults to /opt/robocomp")
    group.add_argument('--doc', action = 'store_true' , help="generate documentation")
    group.add_argument('--installdoc', action = 'store_true' , help="install documentation")
    
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    builder = RCBuild()

    if not args.component or args.component.strip() == '.':
        component_path = os.getcwd()
        
        if builder.ws.path_is_component(component_path):
            #if we are inside a component source directory
            if args.doc or args.installdoc:
                builder.build_docs(component_path, args.installdoc)
            else:
                builder.build_component(component_path, args.install)
        else:
            parser.error(colored(f"{component_path} is not a valid robocomp component directory.", 'red'))
    
    else:
        component = args.component
        if args.doc or args.installdoc:
            builder.build_docs(component, args.installdoc)
        else:
            builder.build_component(component, args.install)

if __name__ == '__main__':
    main()