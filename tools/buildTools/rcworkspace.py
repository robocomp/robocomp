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
    parser.add_argument('-a', '--add', action='store_true', help='Add workspaces searching in the given directory')
    parser.add_argument('-u', '--update', action='store_true', help='Update the components for the existing workspaces')
    parser.add_argument('-d', '--delete', type=str, help='Remove workspaces searching in the given directory')
    parser.add_argument('--clear-all', action='store_true', help='Clear all the information of the workspaces (requires confirmation)')
    parser.add_argument('-l', '--list', action='store_true', help='List the current existing workspaces and components')
    args = parser.parse_args()
    ws = Workspace()
    if args.initialize:
        try:
            ws.update_robocomp_workspaces(args.find)
        except KeyboardInterrupt:
            print("\nCanceled")
    if args.delete:
        try:
            ws.delete_workspace(args.delete)
        except KeyboardInterrupt:
            print("\nCanceled")
    if args.add:
        try:
            ws.add_workspace()
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


    # verify that workspace folder exists
    workspace = os.path.abspath(args.workspace)
    if not os.path.isdir(workspace):
        parser.error('Workspace "%s" does not exist' % workspace)

    #verify that this is not an existing workspace
    if os.path.exists(os.path.join(workspace,".rc_workspace")):
        print('\nWorkspace "%s" is already a workspace' % workspace)
        if WS.register_workspace(workspace):
            print("Re-registered this workspace\n")
        return

    #try creating a workspace
    try:
        WS.init_ws(workspace)
    except Exception as e:
        sys.stderr.write(str(e))
        sys.exit(2)
    else:
        sys.stdout.write("Sucessfully initialized robocomp workspace in %s \n" % (str(os.path.abspath(workspace))) )
        sys.stdout.write("To remove this workspace delete the file .rc_workspace\n")

if __name__ == '__main__':
    main()

