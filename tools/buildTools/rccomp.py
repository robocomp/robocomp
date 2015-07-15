#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
from __future__ import print_function
import os
import argparse, argcomplete
from workspace import workspace as WS

def main():
    parser = argparse.ArgumentParser(description="provides various info about components")
    parser.add_argument('argument', nargs='?', choices=['list'])
    
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    if args.argument=='list':
        components = WS.list_packages(WS.workspace_paths)
        componentsname=[]
        for component in components:
            componentsname.append(component.split('/')[ len(component.split('/')) -1 ])
        opstring = "   ".join(componentsname)
        print(opstring)
    else:
        parser.error("sorry no such option is available ")

if __name__ == '__main__':
    main()