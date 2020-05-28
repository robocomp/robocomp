#!/usr/bin/python

from __future__ import print_function
import os
import sys
import argparse

sys.path.append('/opt/robocomp/python')


from workspace import workspace as WS

def main():
    parser = argparse.ArgumentParser(description="Initialize a robocomp workspace")
    parser.add_argument('-f', '--find', type=str, help='Find possible Robocomp workspaces')
    parser.add_argument('workspace', nargs='?', default='.', help='The path to an existing folder (default: .)')
    args = parser.parse_args()

    if args.find:
        WS.find_robocomp_workspaces(args.find)
    return


    # verify that workspace folder exists
    workspace = os.path.abspath(args.workspace)
    if not os.path.isdir(workspace):
        parser.error('Workspace "%s" does not exist' % workspace)

    #verify that this is not an existing workspace
    if os.path.exists( os.path.join(workspace,".rc_workspace")):
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

