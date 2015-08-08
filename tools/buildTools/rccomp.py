#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
from __future__ import print_function
import os
import argparse, argcomplete
from workspace import workspace as WS

def main():
    parser = argparse.ArgumentParser(description="provides various info about components/workspaces")
    parser.add_argument('argument', nargs='?', choices=['list','listws'])
    
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    if args.argument=='list':
        components = WS.list_packages(WS.workspace_paths)
        componentsname=[]
        for component in components:
            componentsname.append(component.split('/')[ len(component.split('/')) -1 ])
        opstring = "   ".join(componentsname)
        print(opstring)
    elif args.argument == 'listws':
        home = os.path.expanduser("~")
        print("registered workspaces are : \n")
        with open(os.path.join(home,".config/RoboComp/rc_workspace.config")) as f:
            print(f.read())
        print("\tuse 'rc_init_ws' to register a workspace")
    else:
        parser.error("sorry no such option is available ")

if __name__ == '__main__':
    main()