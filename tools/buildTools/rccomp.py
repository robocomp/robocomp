#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
from __future__ import print_function
import os
import argparse, argcomplete
import sys

sys.path.append('/opt/robocomp/python')

from workspace import Workspace

def main():
    parser = argparse.ArgumentParser(description="provides various info about components/workspaces")
    parser.add_argument('argument', nargs='?', choices=['list','listws'])
    
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    ws = Workspace()

    if args.argument=='list':
        try:
            ws.list_workspaces()
        except KeyboardInterrupt:
            print("\nCanceled")
    
    elif args.argument == 'listws':

        print("Registered workspaces are :")
        for workspace in ws.workspace_paths:
            print(f"\t{workspace}")
    else:
        parser.error("sorry no such option is available ")

if __name__ == '__main__':
    main()
