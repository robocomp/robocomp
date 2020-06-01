#!/usr/bin/python

from __future__ import print_function
import os
import sys
import argparse

sys.path.append('/opt/robocomp/python')


from workspace import Workspace

def main():
    parser = argparse.ArgumentParser(description="Initialize a robocomp workspace")
    parser.add_argument('-i', '--initialize', type=str, help='Initialize workspaces searching in the given directory')
    parser.add_argument('-a', '--add', type=str, nargs='?', const=os.getcwd(), help='Add workspaces searching in the given directory')
    parser.add_argument('-u', '--update', action='store_true', help='Update the components for the existing workspaces')
    parser.add_argument('-d', '--delete', type=str, help='Remove workspaces searching in the given directory')
    parser.add_argument('--clear-all', action='store_true', help='Clear all the information of the workspaces (requires confirmation)')
    parser.add_argument('-l', '--list', action='store_true', help='List the current existing workspaces and components')
    args = parser.parse_args()
    ws = Workspace()
    if args.initialize:
        try:
            ws.interactive_workspace_init(args.find)
        except KeyboardInterrupt:
            print("\nCanceled")
    if args.delete:
        try:
            ws.delete_workspace(args.delete)
        except KeyboardInterrupt:
            print("\nCanceled")
    if args.add:
        try:
            ws.add_workspace(args.add)
        except KeyboardInterrupt:
            print("\nCanceled")
    if args.update:
        try:
            ws.update_components_in_workspaces()
        except KeyboardInterrupt:
            print("\nCanceled")
    if args.clear_all:
        try:
            ws.clear_all()
        except KeyboardInterrupt:
            print("\nCanceled")
    if args.list:
        try:
            ws.list_workspaces()
        except KeyboardInterrupt:
            print("\nCanceled")
    return

if __name__ == '__main__':
    main()

